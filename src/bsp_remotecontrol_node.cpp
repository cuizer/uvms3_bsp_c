/**
 * @file bsp_remotecontrol_node.cpp
 * @brief BSP 层开环遥控节点。订阅统一通信节点发布的模式和遥控通道 topic,
 *        做开环推力分配并发布 /hal/thruster/cmd。
 *        模式命令: 1=开启PID, 2=关闭PID, 3=开启键盘, 4=关闭键盘。
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hal/msg/hal_mode_control.hpp"
#include "hal/msg/hal_remote_control.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace {

constexpr double REMOTE_CHANNEL_MIN = 353.0;
constexpr double REMOTE_CHANNEL_MID = 1024.0;
constexpr double REMOTE_CHANNEL_MAX = 1695.0;
constexpr double REMOTE_CHANNEL_HALF_RANGE = REMOTE_CHANNEL_MAX - REMOTE_CHANNEL_MID;
constexpr double REMOTE_CHANNEL_DEADBAND = 20.0;

double normalize_remote_channel(double value)
{
    const double clamped = std::clamp(value, REMOTE_CHANNEL_MIN, REMOTE_CHANNEL_MAX);
    if (std::abs(clamped - REMOTE_CHANNEL_MID) <= REMOTE_CHANNEL_DEADBAND) {
        return 0.0;
    }
    return std::clamp((clamped - REMOTE_CHANNEL_MID) / REMOTE_CHANNEL_HALF_RANGE,
        -1.0, 1.0);
}

}  // namespace

struct RemoteCmd {
    double surge = 0.0;
    double sway = 0.0;
    double heave = 0.0;
    double yaw = 0.0;
    bool fresh = false;
};

class BspRemoteControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    BspRemoteControlNode()
    : LifecycleNode("bsp_remotecontrol_node")
    {
        this->declare_parameter<std::string>("mode_topic", "/hal/modecontrol");
        this->declare_parameter<std::string>("remote_topic", "/hal/remotecontrol");
        // HalModeControl 命令协议:
        //   1 = 开启 PID 控制
        //   2 = 关闭 PID 控制
        //   3 = 开启键盘遥控
        //   4 = 关闭键盘遥控
        this->declare_parameter<int64_t>("mode_pid_enable_value", 1);
        this->declare_parameter<int64_t>("mode_pid_disable_value", 2);
        this->declare_parameter<int64_t>("mode_keyboard_enable_value", 3);
        this->declare_parameter<int64_t>("mode_keyboard_disable_value", 4);
        this->declare_parameter("control_rate_hz", 20.0);
        this->declare_parameter("cmd_timeout_s", 0.5);
        this->declare_parameter("surge_scale", 300.0);
        this->declare_parameter("sway_scale", 160.0);
        this->declare_parameter("heave_scale", 80.0);
        this->declare_parameter("yaw_scale", 5.0);
        this->declare_parameter("smoothing.enable", true);
        this->declare_parameter("smoothing.slew_Fx", 600.0);
        this->declare_parameter("smoothing.slew_Fy", 320.0);
        this->declare_parameter("smoothing.slew_Fz", 160.0);
        this->declare_parameter("smoothing.slew_Mz", 10.0);
        this->declare_parameter("smoothing.thruster_tau", 0.2);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        thruster_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/hal/thruster/cmd", 10);

        const auto qos_cmd = rclcpp::QoS(10).reliable();
        mode_sub_ = this->create_subscription<hal::msg::HalModeControl>(
            this->get_parameter("mode_topic").as_string(), qos_cmd,
            std::bind(&BspRemoteControlNode::mode_callback, this, std::placeholders::_1));
        remote_sub_ = this->create_subscription<hal::msg::HalRemoteControl>(
            this->get_parameter("remote_topic").as_string(), qos_cmd,
            std::bind(&BspRemoteControlNode::remote_callback, this, std::placeholders::_1));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        thruster_pub_->on_activate();

        const double rate = this->get_parameter("control_rate_hz").as_double();
        cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();
        const int64_t mode_pid_enable =
            this->get_parameter("mode_pid_enable_value").as_int();
        const int64_t mode_pid_disable =
            this->get_parameter("mode_pid_disable_value").as_int();
        const int64_t mode_keyboard_enable =
            this->get_parameter("mode_keyboard_enable_value").as_int();
        const int64_t mode_keyboard_disable =
            this->get_parameter("mode_keyboard_disable_value").as_int();

        const std::array<int64_t, 4> mode_values = {
            mode_pid_enable, mode_pid_disable,
            mode_keyboard_enable, mode_keyboard_disable
        };
        const bool mode_range_valid = std::all_of(
            mode_values.begin(), mode_values.end(),
            [](int64_t value) { return value >= 0 && value <= 255; });
        std::array<int64_t, 4> sorted_modes = mode_values;
        std::sort(sorted_modes.begin(), sorted_modes.end());
        const bool mode_unique =
            std::adjacent_find(sorted_modes.begin(), sorted_modes.end()) == sorted_modes.end();
        if (!mode_range_valid || !mode_unique) {
            RCLCPP_ERROR(get_logger(),
                "[BSP_RC] mode command values must be unique and in [0, 255]");
            thruster_pub_->on_deactivate();
            return CallbackReturn::FAILURE;
        }

        pid_enable_value_ = static_cast<uint8_t>(mode_pid_enable);
        pid_disable_value_ = static_cast<uint8_t>(mode_pid_disable);
        keyboard_enable_value_ = static_cast<uint8_t>(mode_keyboard_enable);
        keyboard_disable_value_ = static_cast<uint8_t>(mode_keyboard_disable);
        surge_scale_ = this->get_parameter("surge_scale").as_double();
        sway_scale_ = this->get_parameter("sway_scale").as_double();
        heave_scale_ = this->get_parameter("heave_scale").as_double();
        yaw_scale_ = this->get_parameter("yaw_scale").as_double();
        smoothing_enable_ = this->get_parameter("smoothing.enable").as_bool();
        slew_Fx_ = this->get_parameter("smoothing.slew_Fx").as_double();
        slew_Fy_ = this->get_parameter("smoothing.slew_Fy").as_double();
        slew_Fz_ = this->get_parameter("smoothing.slew_Fz").as_double();
        slew_Mz_ = this->get_parameter("smoothing.slew_Mz").as_double();
        thruster_tau_ = this->get_parameter("smoothing.thruster_tau").as_double();

        const auto valid_nonnegative = [](double value) {
            return std::isfinite(value) && value >= 0.0;
        };
        if (!std::isfinite(rate) || rate <= 0.0 || rate > 1000.0 ||
            !std::isfinite(cmd_timeout_s_) || cmd_timeout_s_ <= 0.0 ||
            !valid_nonnegative(surge_scale_) || !valid_nonnegative(sway_scale_) ||
            !valid_nonnegative(heave_scale_) || !valid_nonnegative(yaw_scale_) ||
            !valid_nonnegative(slew_Fx_) || !valid_nonnegative(slew_Fy_) ||
            !valid_nonnegative(slew_Fz_) || !valid_nonnegative(slew_Mz_) ||
            !valid_nonnegative(thruster_tau_)) {
            RCLCPP_ERROR(get_logger(), "[BSP_RC] invalid control parameter");
            thruster_pub_->on_deactivate();
            return CallbackReturn::FAILURE;
        }

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            latest_cmd_ = RemoteCmd{};
            last_cmd_time_ = std::chrono::steady_clock::now();
            remote_enabled_ = false;
        }
        reset_smoothing();
        watchdog_tripped_ = false;

        const int period_ms = std::max(1, static_cast<int>(1000.0 / rate));
        ctrl_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
            std::bind(&BspRemoteControlNode::control_loop, this));

        RCLCPP_INFO(get_logger(),
            "[BSP_RC] activated, input=(%s, %s), rate=%.1fHz",
            this->get_parameter("mode_topic").as_string().c_str(),
            this->get_parameter("remote_topic").as_string().c_str(), rate);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (ctrl_timer_) ctrl_timer_->cancel();
        const bool was_enabled = disable_keyboard_output();
        if (was_enabled) {
            send_zero_thrust();
        }
        thruster_pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        mode_sub_.reset();
        remote_sub_.reset();
        ctrl_timer_.reset();
        thruster_pub_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
    {
        if (ctrl_timer_) ctrl_timer_->cancel();
        const bool was_enabled = disable_keyboard_output();
        if (was_enabled) {
            send_zero_thrust();
        }
        return CallbackReturn::SUCCESS;
    }

private:
    void mode_callback(const hal::msg::HalModeControl::SharedPtr msg)
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) return;

        const uint8_t cmd = msg->modecontrol_cmd;

        // 3: 开启键盘遥控。
        if (cmd == keyboard_enable_value_) {
            bool mode_changed = false;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                if (!remote_enabled_) {
                    remote_enabled_ = true;
                    latest_cmd_ = RemoteCmd{};
                    last_cmd_time_ = std::chrono::steady_clock::now();
                    mode_changed = true;
                }
            }
            if (mode_changed) {
                reset_smoothing();
                watchdog_tripped_ = false;
                RCLCPP_INFO(get_logger(),
                    "[BSP_RC] keyboard remote enabled (cmd=%u)",
                    static_cast<unsigned>(cmd));
            }
            return;
        }

        // 4: 显式关闭键盘遥控。
        // 1: 开启 PID，PID 优先接管，因此键盘节点必须自动退出。
        if (cmd == keyboard_disable_value_ || cmd == pid_enable_value_) {
            const bool was_enabled = disable_keyboard_output();
            if (was_enabled) {
                send_zero_thrust();  // 仅在退出瞬间发布一次零推力
                RCLCPP_INFO(get_logger(),
                    "[BSP_RC] keyboard remote disabled (cmd=%u%s)",
                    static_cast<unsigned>(cmd),
                    cmd == pid_enable_value_ ? ", PID takeover" : "");
            }
            return;
        }

        // 2 = 关闭 PID，与键盘模式状态无关，因此这里不动作。
        if (cmd == pid_disable_value_) {
            return;
        }

        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "[BSP_RC] unknown mode command: %u", static_cast<unsigned>(cmd));
    }

    bool disable_keyboard_output()
    {
        bool was_enabled = false;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            was_enabled = remote_enabled_;
            remote_enabled_ = false;
            latest_cmd_ = RemoteCmd{};
            last_cmd_time_ = std::chrono::steady_clock::now();
        }
        if (was_enabled) {
            reset_smoothing();
            watchdog_tripped_ = false;
        }
        return was_enabled;
    }

    void remote_callback(const hal::msg::HalRemoteControl::SharedPtr msg)
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) return;

        const std::array<double, 6> channels = {
            static_cast<double>(msg->tunnel1_para),
            static_cast<double>(msg->tunnel2_para),
            static_cast<double>(msg->tunnel3_para),
            static_cast<double>(msg->tunnel4_para),
            static_cast<double>(msg->tunnel5_para),
            static_cast<double>(msg->tunnel6_para),
        };
        if (!std::all_of(channels.begin(), channels.end(),
                [](double value) { return std::isfinite(value); })) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[BSP_RC] /hal/remotecontrol contains non-finite channel");
            return;
        }

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (!remote_enabled_) return;

        latest_cmd_.surge = normalize_remote_channel(channels[0]);
        latest_cmd_.sway = normalize_remote_channel(channels[1]);
        latest_cmd_.heave = normalize_remote_channel(channels[2]);
        latest_cmd_.yaw = normalize_remote_channel(channels[3]);
        latest_cmd_.fresh = true;
        last_cmd_time_ = std::chrono::steady_clock::now();
    }

    void control_loop()
    {
        RemoteCmd cmd;
        bool enabled = false;
        std::chrono::steady_clock::time_point last_cmd_time;
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            cmd = latest_cmd_;
            enabled = remote_enabled_;
            last_cmd_time = last_cmd_time_;
        }

        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_cmd_time).count();
        // 未开启键盘模式时完全不发布，避免与 PID 节点争抢 /hal/thruster/cmd。
        if (!enabled) {
            return;
        }

        // 键盘模式已开启，但没有新指令或指令超时：安全输出零推力。
        if (!cmd.fresh || age > cmd_timeout_s_) {
            if (cmd.fresh && !watchdog_tripped_) {
                RCLCPP_WARN(get_logger(),
                    "[BSP_RC] command watchdog triggered (age=%.3fs)", age);
                watchdog_tripped_ = true;
            }
            send_zero_thrust();
            reset_smoothing();
            return;
        }
        watchdog_tripped_ = false;

        const double Fx_raw = cmd.surge * surge_scale_;
        const double Fy_raw = cmd.sway * sway_scale_;
        const double Fz_raw = cmd.heave * heave_scale_;
        const double Mz_raw = cmd.yaw * yaw_scale_;
        const double dt = 1.0 / this->get_parameter("control_rate_hz").as_double();
        double tau_smooth[6] = {Fx_raw, Fy_raw, Fz_raw, 0.0, 0.0, Mz_raw};

        if (smoothing_enable_ && dt > 0.0) {
            const double slew[6] = {slew_Fx_, slew_Fy_, slew_Fz_, 0.0, 0.0, slew_Mz_};
            for (int i = 0; i < 6; i++) {
                const double max_step = slew[i] * dt;
                const double diff = tau_smooth[i] - prev_tau_[i];
                if (diff > max_step) tau_smooth[i] = prev_tau_[i] + max_step;
                if (diff < -max_step) tau_smooth[i] = prev_tau_[i] - max_step;
            }
        }
        for (int i = 0; i < 6; i++) prev_tau_[i] = tau_smooth[i];

        std::array<double, 6> f_raw{};
        for (int i = 0; i < 6; i++) {
            double sum = 0.0;
            for (int j = 0; j < 6; j++) sum += T_PINV[i][j] * tau_smooth[j];
            f_raw[i] = sum;
        }

        std::array<double, 6> f_out{};
        if (smoothing_enable_ && thruster_tau_ > 0.0) {
            const double alpha = dt / (thruster_tau_ + dt);
            for (int i = 0; i < 6; i++) {
                thruster_filtered_[i] =
                    alpha * f_raw[i] + (1.0 - alpha) * thruster_filtered_[i];
                f_out[i] = thruster_filtered_[i];
            }
        } else {
            f_out = f_raw;
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data.reserve(6);
        for (int i = 0; i < 6; i++) {
            msg.data.push_back(std::clamp(f_out[i], -100.0, 100.0));
        }
        thruster_pub_->publish(msg);
    }

    void send_zero_thrust()
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) return;
        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        thruster_pub_->publish(msg);
    }

    void reset_smoothing()
    {
        prev_tau_ = {0, 0, 0, 0, 0, 0};
        thruster_filtered_ = {0, 0, 0, 0, 0, 0};
    }

    static constexpr std::array<std::array<double, 6>, 6> T_PINV = {{
        {  0.74226804, -0.18556701,  0.00000000,  0.00000000,  0.00000000,  0.41237113 },
        {  0.00000000,  0.00000000,  0.45871560,  5.00000000,  0.13761468, -0.00000000 },
        {  0.00000000, -0.00000000,  0.45871560, -5.00000000,  0.13761468,  0.00000000 },
        { -0.10309278,  0.52577320, -0.00000000,  0.00000000, -0.00000000,  2.16494845 },
        {  0.10309278,  0.47422680,  0.00000000, -0.00000000,  0.00000000, -2.16494845 },
        {  0.41237113, -0.10309278, -0.00000000, -0.00000000, -0.00000000,  1.34020619 }
    }};

    rclcpp_lifecycle::LifecyclePublisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
    rclcpp::Subscription<hal::msg::HalModeControl>::SharedPtr mode_sub_;
    rclcpp::Subscription<hal::msg::HalRemoteControl>::SharedPtr remote_sub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;

    std::mutex cmd_mutex_;
    RemoteCmd latest_cmd_;
    std::chrono::steady_clock::time_point last_cmd_time_{};
    bool remote_enabled_ = false;
    bool watchdog_tripped_ = false;
    uint8_t pid_enable_value_ = 1;
    uint8_t pid_disable_value_ = 2;
    uint8_t keyboard_enable_value_ = 3;
    uint8_t keyboard_disable_value_ = 4;

    double cmd_timeout_s_ = 0.5;
    double surge_scale_ = 300.0;
    double sway_scale_ = 160.0;
    double heave_scale_ = 80.0;
    double yaw_scale_ = 5.0;
    bool smoothing_enable_ = true;
    double slew_Fx_ = 600.0;
    double slew_Fy_ = 320.0;
    double slew_Fz_ = 160.0;
    double slew_Mz_ = 10.0;
    double thruster_tau_ = 0.2;
    std::array<double, 6> prev_tau_{};
    std::array<double, 6> thruster_filtered_{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BspRemoteControlNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node->get_node_base_interface());
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
