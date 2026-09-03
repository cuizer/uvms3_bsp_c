/**
 * @file bsp_remotecontrol_node.cpp
 * @brief BSP 层键盘/遥控开环接管节点。订阅统一通信节点发布的模式和遥控通道 topic,
 *        按与 bsp_motioncontrol_node 一致的物理单位分配矩阵做开环推力分配并发布
 *        /hal/thruster/cmd。
 *        输出顺序: data[0]=主推 CAN ID 0x301, data[1..5]=辅推电调 ID2..ID6。
 *        模式命令: 1=开启PID, 2=关闭PID, 3=开启键盘, 4=关闭键盘。
 *
 *        通道语义:
 *        - tunnel1: surge 前后开环力 Fx (N)
 *        - tunnel2: sway 左右开环力 Fy (N)
 *        - tunnel3: 定深关闭时为 heave 垂向开环力 Fz (N);
 *                   定深开启时按 353..1695、中位 1024 归一化为深度目标增减速率
 *        - tunnel4: yaw 开环转艏力矩 Mz (N·m), 不是艏向角 PID
 *        - tunnel5: 定深开关, 0=关闭, 1=开启
 *        艏向角闭环由 bsp_motioncontrol_node 在 PID 模式消费 /hal/remotecontrol 完成。
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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hal/msg/hal_mode_control.hpp"
#include "hal/msg/hal_remote_control.hpp"
#include "hal/msg/hal_depthsensor.hpp"

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace {

constexpr double REMOTE_CHANNEL_MIN = 353.0;
constexpr double REMOTE_CHANNEL_MID = 1024.0;
constexpr double REMOTE_CHANNEL_MAX = 1695.0;
constexpr double REMOTE_CHANNEL_HALF_RANGE = REMOTE_CHANNEL_MAX - REMOTE_CHANNEL_MID;
constexpr double REMOTE_CHANNEL_DEADBAND = 20.0;
constexpr double BINARY_SWITCH_MIN = -0.5;
constexpr double BINARY_SWITCH_MAX = 1.5;

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
    double depth_axis = 0.0;
    double yaw = 0.0;
    bool depth_hold = false;
    bool fresh = false;
};

struct PidController {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double max_i = 0.0;
    double max_out = 0.0;

    double integral = 0.0;
    double prev_error = 0.0;
    bool has_prev_error = false;

    double update(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;

        const double p_term = kp * error;
        double i_term = 0.0;
        if (ki > 0.0 && max_i > 0.0) {
            integral += ki * error * dt;
            integral = std::clamp(integral, -max_i, max_i);
            i_term = integral;
        }

        double d_term = 0.0;
        if (kd > 0.0 && has_prev_error) {
            d_term = kd * (error - prev_error) / dt;
        }
        prev_error = error;
        has_prev_error = true;

        double out = p_term + i_term + d_term;
        if (max_out > 0.0) {
            out = std::clamp(out, -max_out, max_out);
        }
        return out;
    }

    void reset()
    {
        integral = 0.0;
        prev_error = 0.0;
        has_prev_error = false;
    }
};

class BspRemoteControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    BspRemoteControlNode()
    : LifecycleNode("bsp_remotecontrol_node")
    {
        this->declare_parameter<std::string>("mode_topic", "/hal/modecontrol");
        this->declare_parameter<std::string>("remote_topic", "/hal/remotecontrol");
        this->declare_parameter<std::string>("depth_topic", "/hal/depthsensor");
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
        this->declare_parameter("depth_hold.switch_threshold", 0.5);
        this->declare_parameter("depth_hold.rate_mps", 0.2);
        this->declare_parameter("depth_hold.min_depth_m", 0.0);
        this->declare_parameter("depth_hold.max_depth_m", 10.0);
        this->declare_parameter("depth_hold.axis_sign", 1.0);
        this->declare_parameter("depth_hold.pid.kp", 80.0);
        this->declare_parameter("depth_hold.pid.ki", 5.0);
        this->declare_parameter("depth_hold.pid.kd", 20.0);
        this->declare_parameter("depth_hold.pid.max_i", 30.0);
        this->declare_parameter("depth_hold.pid.max_out", 120.0);
        this->declare_parameter("alloc_lambda", 0.01);
        this->declare_parameter("deadzone_pct", 3.0);
        this->declare_parameter<std::vector<double>>("thrust_limits",
            {441.0, 69.0, 69.0, 69.0, 69.0, 69.0});
        this->declare_parameter<double>("thrust_max_pct", 100.0);
        this->declare_parameter<double>("thrust_min_pct", -100.0);
        // T ∈ R^(6×6): tau = T * u.
        // 列/输出顺序: [主推0x301, 辅推ID2, 辅推ID3, 辅推ID4, 辅推ID5, 辅推ID6]
        // 行顺序: [Fx, Fy, Fz, Mx, My, Mz]
        this->declare_parameter<std::vector<double>>("alloc_matrix", {
            1.0, 0.0, 0.0, 0.0, 0.5, 0.5,
            0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
            0.0, 1.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.1, -0.1, 0.0, 0.0, 0.0,
            0.0, 0.3, 0.3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.2, -0.2, 0.1,
        });
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
        depth_sub_ = this->create_subscription<hal::msg::HalDepthsensor>(
            this->get_parameter("depth_topic").as_string(), rclcpp::QoS(10).best_effort(),
            std::bind(&BspRemoteControlNode::depth_callback, this, std::placeholders::_1));

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
        depth_hold_switch_threshold_ =
            this->get_parameter("depth_hold.switch_threshold").as_double();
        depth_hold_rate_mps_ =
            this->get_parameter("depth_hold.rate_mps").as_double();
        depth_hold_min_m_ =
            this->get_parameter("depth_hold.min_depth_m").as_double();
        depth_hold_max_m_ =
            this->get_parameter("depth_hold.max_depth_m").as_double();
        depth_hold_axis_sign_ =
            this->get_parameter("depth_hold.axis_sign").as_double();
        load_pid_params("depth_hold.pid", depth_pid_);
        alloc_lambda_ = this->get_parameter("alloc_lambda").as_double();
        deadzone_pct_ = this->get_parameter("deadzone_pct").as_double();
        thrust_max_pct_ = this->get_parameter("thrust_max_pct").as_double();
        thrust_min_pct_ = this->get_parameter("thrust_min_pct").as_double();
        alloc_vec_ = this->get_parameter("alloc_matrix").as_double_array();
        const auto thrust_limits = this->get_parameter("thrust_limits").as_double_array();
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
            !std::isfinite(depth_hold_switch_threshold_) ||
            depth_hold_switch_threshold_ < 0.0 ||
            depth_hold_switch_threshold_ > 1.0 ||
            !valid_nonnegative(depth_hold_rate_mps_) ||
            !std::isfinite(depth_hold_min_m_) ||
            !std::isfinite(depth_hold_max_m_) ||
            depth_hold_min_m_ > depth_hold_max_m_ ||
            !std::isfinite(depth_hold_axis_sign_) ||
            std::abs(depth_hold_axis_sign_) > 10.0 ||
            !valid_pid(depth_pid_) ||
            !valid_nonnegative(slew_Fx_) || !valid_nonnegative(slew_Fy_) ||
            !valid_nonnegative(slew_Fz_) || !valid_nonnegative(slew_Mz_) ||
            !valid_nonnegative(thruster_tau_) ||
            !std::isfinite(alloc_lambda_) || alloc_lambda_ <= 0.0 ||
            alloc_lambda_ > 1000.0 ||
            !std::isfinite(deadzone_pct_) || deadzone_pct_ < 0.0 ||
            deadzone_pct_ > 100.0 ||
            !std::isfinite(thrust_min_pct_) || !std::isfinite(thrust_max_pct_) ||
            thrust_min_pct_ < -100.0 || thrust_max_pct_ > 100.0 ||
            thrust_min_pct_ > thrust_max_pct_) {
            RCLCPP_ERROR(get_logger(), "[BSP_RC] invalid control parameter");
            thruster_pub_->on_deactivate();
            return CallbackReturn::FAILURE;
        }
        if (alloc_vec_.size() != 36U ||
            !std::all_of(alloc_vec_.begin(), alloc_vec_.end(),
                [](double value) { return std::isfinite(value); })) {
            RCLCPP_ERROR(get_logger(),
                "[BSP_RC] alloc_matrix must contain exactly 36 finite elements");
            thruster_pub_->on_deactivate();
            return CallbackReturn::FAILURE;
        }
        if (thrust_limits.size() != 6U ||
            !std::all_of(thrust_limits.begin(), thrust_limits.end(),
                [](double value) {
                    return std::isfinite(value) && value > 0.0 && value <= 1e6;
                })) {
            RCLCPP_ERROR(get_logger(),
                "[BSP_RC] thrust_limits must contain 6 values in (0, 1e6]");
            thruster_pub_->on_deactivate();
            return CallbackReturn::FAILURE;
        }
        for (size_t i = 0; i < 6U; ++i) {
            thrust_limits_[i] = thrust_limits[i];
        }
        control_period_s_ = 1.0 / rate;

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            latest_cmd_ = RemoteCmd{};
            last_cmd_time_ = std::chrono::steady_clock::now();
            remote_enabled_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            depth_m_ = 0.0;
            depth_valid_ = false;
        }
        reset_depth_hold();
        reset_smoothing();
        watchdog_tripped_ = false;

        const int period_ms = std::max(1, static_cast<int>(1000.0 / rate));
        ctrl_timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
            std::bind(&BspRemoteControlNode::control_loop, this));

        RCLCPP_INFO(get_logger(),
            "[BSP_RC] activated, input=(%s, %s), depth=%s, rate=%.1fHz, alloc=DLS",
            this->get_parameter("mode_topic").as_string().c_str(),
            this->get_parameter("remote_topic").as_string().c_str(),
            this->get_parameter("depth_topic").as_string().c_str(), rate);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (ctrl_timer_) ctrl_timer_->cancel();
        const bool was_enabled = disable_keyboard_output();
        if (was_enabled) {
            send_zero_thrust();
        }
        reset_depth_hold();
        thruster_pub_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
    {
        mode_sub_.reset();
        remote_sub_.reset();
        depth_sub_.reset();
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
        reset_depth_hold();
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
            reset_depth_hold();
            watchdog_tripped_ = false;
        }
        return was_enabled;
    }

    void depth_callback(const hal::msg::HalDepthsensor::SharedPtr msg)
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) return;

        std::lock_guard<std::mutex> lock(state_mutex_);
        depth_m_ = static_cast<double>(msg->depth_avg);
        depth_valid_ = (msg->connection_status == 1) && std::isfinite(depth_m_);
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
        latest_cmd_.depth_axis = normalize_remote_channel(channels[2]);
        latest_cmd_.yaw = normalize_remote_channel(channels[3]);
        if (channels[4] < BINARY_SWITCH_MIN || channels[4] > BINARY_SWITCH_MAX) {
            latest_cmd_.depth_hold = false;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "[BSP_RC] tunnel5 depth hold switch expects 0/1, got %.3f",
                channels[4]);
        } else {
            latest_cmd_.depth_hold = channels[4] >= depth_hold_switch_threshold_;
        }
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
            reset_depth_hold();
            return;
        }
        watchdog_tripped_ = false;

        const double Fx_raw = cmd.surge * surge_scale_;
        const double Fy_raw = cmd.sway * sway_scale_;
        const double Mz_raw = cmd.yaw * yaw_scale_;
        const double dt = control_period_s_;
        double Fz_raw = cmd.depth_axis * heave_scale_;
        if (cmd.depth_hold) {
            double depth = 0.0;
            bool depth_valid = false;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                depth = depth_m_;
                depth_valid = depth_valid_;
            }

            if (!depth_valid) {
                if (depth_hold_active_) {
                    RCLCPP_WARN(get_logger(),
                        "[BSP_RC] depth hold disabled: invalid depth sensor");
                } else {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                        "[BSP_RC] depth hold requested but depth sensor is invalid");
                }
                reset_depth_hold();
                Fz_raw = 0.0;
            } else {
                if (!depth_hold_active_) {
                    depth_hold_active_ = true;
                    depth_hold_target_m_ =
                        std::clamp(depth, depth_hold_min_m_, depth_hold_max_m_);
                    depth_pid_.reset();
                    RCLCPP_INFO(get_logger(),
                        "[BSP_RC] depth hold enabled: target=%.3fm",
                        depth_hold_target_m_);
                }

                depth_hold_target_m_ = std::clamp(
                    depth_hold_target_m_ +
                        depth_hold_axis_sign_ * cmd.depth_axis *
                            depth_hold_rate_mps_ * dt,
                    depth_hold_min_m_,
                    depth_hold_max_m_);
                Fz_raw = depth_pid_.update(depth_hold_target_m_ - depth, dt);
            }
        } else if (depth_hold_active_) {
            RCLCPP_INFO(get_logger(), "[BSP_RC] depth hold disabled");
            reset_depth_hold();
        }
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

        const std::array<double, 6> tau_cmd = {
            tau_smooth[0], tau_smooth[1], tau_smooth[2],
            tau_smooth[3], tau_smooth[4], tau_smooth[5]};
        std::array<double, 6> f_raw = allocate_thrust(tau_cmd);

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
            const double u_eff =
                (std::abs(f_out[i]) < deadzone_pct_ * 0.01 * thrust_limits_[i])
                    ? 0.0 : f_out[i];
            msg.data.push_back(std::clamp(u_eff / thrust_limits_[i] * 100.0,
                thrust_min_pct_, thrust_max_pct_));
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

    void reset_depth_hold()
    {
        depth_hold_active_ = false;
        depth_hold_target_m_ = 0.0;
        depth_pid_.reset();
    }

    void load_pid_params(const std::string & prefix, PidController & pid)
    {
        pid.kp = this->get_parameter(prefix + ".kp").as_double();
        pid.ki = this->get_parameter(prefix + ".ki").as_double();
        pid.kd = this->get_parameter(prefix + ".kd").as_double();
        pid.max_i = this->get_parameter(prefix + ".max_i").as_double();
        pid.max_out = this->get_parameter(prefix + ".max_out").as_double();
        pid.reset();
    }

    static bool valid_pid(const PidController & pid)
    {
        return std::isfinite(pid.kp) && std::isfinite(pid.ki) &&
            std::isfinite(pid.kd) && std::isfinite(pid.max_i) &&
            std::isfinite(pid.max_out) && pid.kp >= 0.0 &&
            pid.ki >= 0.0 && pid.kd >= 0.0 && pid.max_i >= 0.0 &&
            pid.max_out > 0.0;
    }

    // 与 bsp_motioncontrol_node 保持一致: 逐次截断阻尼最小二乘分配。
    std::array<double, 6> allocate_thrust(const std::array<double, 6> & tau)
    {
        std::array<double, 6> u{};
        std::array<bool, 6> pinned{};
        const double lambda = (std::isfinite(alloc_lambda_) && alloc_lambda_ > 1e-9)
            ? alloc_lambda_ : 0.01;

        for (int iter = 0; iter < 6; ++iter) {
            int n_free = 0;
            int free_idx[6];
            for (int k = 0; k < 6; ++k) {
                if (!pinned[k]) free_idx[n_free++] = k;
            }
            if (n_free == 0) break;

            std::array<double, 6> tau_rem = tau;
            for (int k = 0; k < 6; ++k) {
                if (!pinned[k]) continue;
                for (int i = 0; i < 6; ++i) {
                    tau_rem[i] -= alloc_vec_[i * 6 + k] * u[k];
                }
            }

            double M[6][6] = {};
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    double sum = 0.0;
                    for (int n = 0; n < n_free; ++n) {
                        const int k = free_idx[n];
                        sum += alloc_vec_[i * 6 + k] * alloc_vec_[j * 6 + k];
                    }
                    M[i][j] = sum;
                    if (i == j) M[i][j] += lambda * lambda;
                }
            }

            double L[6][6] = {};
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j <= i; ++j) {
                    double sum = M[i][j];
                    for (int k = 0; k < j; ++k) sum -= L[i][k] * L[j][k];
                    if (i == j) L[i][j] = std::sqrt(std::max(sum, 1e-12));
                    else L[i][j] = sum / L[j][j];
                }
            }

            double y[6] = {};
            double z[6] = {};
            for (int i = 0; i < 6; ++i) {
                double sum = tau_rem[i];
                for (int j = 0; j < i; ++j) sum -= L[i][j] * z[j];
                z[i] = sum / L[i][i];
            }
            for (int i = 5; i >= 0; --i) {
                double sum = z[i];
                for (int j = i + 1; j < 6; ++j) sum -= L[j][i] * y[j];
                y[i] = sum / L[i][i];
            }

            bool any_sat = false;
            for (int j = 0; j < 6; ++j) {
                if (pinned[j]) continue;
                double val = 0.0;
                for (int i = 0; i < 6; ++i) {
                    val += alloc_vec_[i * 6 + j] * y[i];
                }
                const double lim = thrust_limits_[j];
                if (val > lim) {
                    u[j] = lim;
                    pinned[j] = true;
                    any_sat = true;
                } else if (val < -lim) {
                    u[j] = -lim;
                    pinned[j] = true;
                    any_sat = true;
                } else {
                    u[j] = val;
                }
            }
            if (!any_sat) break;
        }

        for (int k = 0; k < 6; ++k) {
            u[k] = std::clamp(u[k], -thrust_limits_[k], thrust_limits_[k]);
        }
        return u;
    }

    rclcpp_lifecycle::LifecyclePublisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr thruster_pub_;
    rclcpp::Subscription<hal::msg::HalModeControl>::SharedPtr mode_sub_;
    rclcpp::Subscription<hal::msg::HalRemoteControl>::SharedPtr remote_sub_;
    rclcpp::Subscription<hal::msg::HalDepthsensor>::SharedPtr depth_sub_;
    rclcpp::TimerBase::SharedPtr ctrl_timer_;

    std::mutex cmd_mutex_;
    std::mutex state_mutex_;
    RemoteCmd latest_cmd_;
    std::chrono::steady_clock::time_point last_cmd_time_{};
    double depth_m_ = 0.0;
    bool depth_valid_ = false;
    bool remote_enabled_ = false;
    bool watchdog_tripped_ = false;
    bool depth_hold_active_ = false;
    uint8_t pid_enable_value_ = 1;
    uint8_t pid_disable_value_ = 2;
    uint8_t keyboard_enable_value_ = 3;
    uint8_t keyboard_disable_value_ = 4;

    double cmd_timeout_s_ = 0.5;
    double control_period_s_ = 0.05;
    double surge_scale_ = 300.0;
    double sway_scale_ = 160.0;
    double heave_scale_ = 80.0;
    double yaw_scale_ = 5.0;
    double depth_hold_switch_threshold_ = 0.5;
    double depth_hold_rate_mps_ = 0.2;
    double depth_hold_min_m_ = 0.0;
    double depth_hold_max_m_ = 10.0;
    double depth_hold_axis_sign_ = 1.0;
    double depth_hold_target_m_ = 0.0;
    PidController depth_pid_;
    double alloc_lambda_ = 0.01;
    double deadzone_pct_ = 3.0;
    double thrust_max_pct_ = 100.0;
    double thrust_min_pct_ = -100.0;
    std::array<double, 6> thrust_limits_{441.0, 69.0, 69.0, 69.0, 69.0, 69.0};
    std::vector<double> alloc_vec_;
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
