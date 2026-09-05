/**
 * @file bsp_remotecontrol_node.cpp
 * @brief BSP 层键盘/遥控开环推力控制节点。
 *
 * 功能:
 *   1. 订阅 /hal/modecontrol 与 /hal/remotecontrol；
 *   2. 键盘/遥控模式下，将遥控量直接转换为体坐标系广义力/力矩；
 *   3. 使用与 bsp_motioncontrol_node 相同的推进器真实布局和 DLS 分配算法；
 *   4. 发布 /hal/thruster/cmd。
 *
 * 输出顺序:
 *   data[0] = 主推 CAN ID 0x301
 *   data[1] = 辅推 ID2
 *   data[2] = 辅推 ID3
 *   data[3] = 辅推 ID4
 *   data[4] = 辅推 ID5
 *   data[5] = 辅推 ID6
 *
 * 模式命令:
 *   1 = 开启 PID 控制（遥控节点自动退出）
 *   2 = 关闭 PID 控制
 *   3 = 开启键盘/遥控开环控制
 *   4 = 关闭键盘/遥控开环控制
 *
 * 遥控通道:
 *   tunnel1 -> surge: Fx 开环纵向力
 *   tunnel2 -> sway : Fy 开环横向力
 *   tunnel3 -> heave: Fz 开环垂向力（不再有定深功能）
 *   tunnel4 -> yaw  : Mz 开环转艏力矩（不做艏向 PID）
 *   tunnel5/tunnel6 当前保留，不参与推进器计算。
 *
 * 坐标约定与 bsp_motioncontrol_node 一致:
 *   x 向前为正，z 向下为正；
 *   正推进器指令表示该推进器沿其定义的体轴正方向产生推力。
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
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

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace {

constexpr double REMOTE_CHANNEL_MIN = 353.0;
constexpr double REMOTE_CHANNEL_MID = 1024.0;
constexpr double REMOTE_CHANNEL_MAX = 1695.0;
constexpr double REMOTE_CHANNEL_HALF_RANGE =
    REMOTE_CHANNEL_MAX - REMOTE_CHANNEL_MID;
constexpr double REMOTE_CHANNEL_DEADBAND = 20.0;

double normalize_remote_channel(double value)
{
    const double clamped =
        std::clamp(value, REMOTE_CHANNEL_MIN, REMOTE_CHANNEL_MAX);

    if (std::abs(clamped - REMOTE_CHANNEL_MID) <= REMOTE_CHANNEL_DEADBAND) {
        return 0.0;
    }

    return std::clamp(
        (clamped - REMOTE_CHANNEL_MID) / REMOTE_CHANNEL_HALF_RANGE,
        -1.0, 1.0);
}

}  // namespace

struct RemoteCmd
{
    double surge = 0.0;  // [-1, 1]
    double sway  = 0.0;  // [-1, 1]
    double heave = 0.0;  // [-1, 1]
    double yaw   = 0.0;  // [-1, 1]
    bool fresh   = false;
};

class BspRemoteControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    BspRemoteControlNode()
    : LifecycleNode("bsp_remotecontrol_node")
    {
        // ---------------------------------------------------------------------
        // ROS topic / mode protocol
        // ---------------------------------------------------------------------
        this->declare_parameter<std::string>("mode_topic", "/hal/modecontrol");
        this->declare_parameter<std::string>("remote_topic", "/hal/remotecontrol");

        this->declare_parameter<int64_t>("mode_pid_enable_value", 1);
        this->declare_parameter<int64_t>("mode_pid_disable_value", 2);
        this->declare_parameter<int64_t>("mode_keyboard_enable_value", 3);
        this->declare_parameter<int64_t>("mode_keyboard_disable_value", 4);

        // 与 motioncontrol 节点统一控制周期/看门狗默认值
        this->declare_parameter<double>("control_rate_hz", 50.0);
        this->declare_parameter<double>("cmd_timeout_s", 1.0);

        // ---------------------------------------------------------------------
        // 遥控专用：归一化摇杆 [-1,1] -> 广义力/力矩
        // 这些不是 PID 参数，因此保留为遥控节点独立标定项。
        // ---------------------------------------------------------------------
        this->declare_parameter<double>("surge_scale", 300.0);  // N
        this->declare_parameter<double>("sway_scale",  160.0);  // N
        this->declare_parameter<double>("heave_scale",  80.0);  // N
        this->declare_parameter<double>("yaw_scale",    50.0);  // N*m

        // ---------------------------------------------------------------------
        // 与 bsp_motioncontrol_node 一致的执行器参数
        // ---------------------------------------------------------------------
        this->declare_parameter<double>("alloc_lambda", 0.01);
        this->declare_parameter<double>("deadzone_pct", 3.0);

        this->declare_parameter<std::vector<double>>(
            "thrust_limits",
            {441.0, 69.0, 69.0, 69.0, 69.0, 69.0});

        this->declare_parameter<double>("thrust_max_pct",  100.0);
        this->declare_parameter<double>("thrust_min_pct", -100.0);

        // ---------------------------------------------------------------------
        // 控制分配矩阵 T: tau = T * u
        // 行: [Fx, Fy, Fz, Mx, My, Mz]
        // 列: [Main, ID2, ID3, ID4, ID5, ID6]
        //
        // 真实布局与 bsp_motioncontrol_node 一致:
        // Main: 主推
        // ID2 : 后侧推, x=-1.78283 m
        // ID3 : 后垂推, x=-1.6631 m, y=-0.075 m
        // ID4 : 后垂推, x=-1.6631 m, y=+0.075 m
        // ID5 : 前垂推, x=+1.6236 m
        // ID6 : 前侧推, x=+1.74559 m
        //
        // 力矩: tau = r x F
        //   Mx = y*Fz
        //   My = -x*Fz
        //   Mz = x*Fy
        //
        // 若某个推进器实测正指令产生的物理推力方向与定义相反，
        // 必须把该推进器对应的“整列”全部乘 -1，不能只改某一行。
        // ---------------------------------------------------------------------
        const std::vector<double> default_T = {
            // Main      ID2        ID3       ID4       ID5       ID6
             1.0,       0.0,       0.0,      0.0,      0.0,      0.0,       // Fx
             0.0,       1.0,       0.0,      0.0,      0.0,      1.0,       // Fy
             0.0,       0.0,       1.0,      1.0,      1.0,      0.0,       // Fz
             0.0,       0.0,      -0.075,    0.075,    0.0,      0.0,       // Mx
             0.0,       0.0,       1.6631,   1.6631,  -1.6236,   0.0,       // My
             0.0,      -1.78283,   0.0,      0.0,      0.0,      1.74559   // Mz
        };
        this->declare_parameter<std::vector<double>>("alloc_matrix", default_T);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "[BSP_RC] on_configure");

        thruster_pub_ =
            this->create_publisher<std_msgs::msg::Float64MultiArray>(
                "/hal/thruster/cmd", 10);

        const auto qos_cmd = rclcpp::QoS(10).reliable();

        mode_sub_ = this->create_subscription<hal::msg::HalModeControl>(
            this->get_parameter("mode_topic").as_string(),
            qos_cmd,
            std::bind(
                &BspRemoteControlNode::mode_callback,
                this,
                std::placeholders::_1));

        remote_sub_ = this->create_subscription<hal::msg::HalRemoteControl>(
            this->get_parameter("remote_topic").as_string(),
            qos_cmd,
            std::bind(
                &BspRemoteControlNode::remote_callback,
                this,
                std::placeholders::_1));

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
        RCLCPP_INFO(get_logger(), "[BSP_RC] on_activate");

        thruster_pub_->on_activate();

        if (!load_parameters()) {
            thruster_pub_->on_deactivate();
            return CallbackReturn::FAILURE;
        }

        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            latest_cmd_ = RemoteCmd{};
            last_cmd_time_ = std::chrono::steady_clock::now();
            remote_enabled_ = false;
        }

        watchdog_tripped_ = false;

        const double rate = 1.0 / control_period_s_;
        const int period_ms =
            std::max(1, static_cast<int>(1000.0 / rate));

        ctrl_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&BspRemoteControlNode::control_loop, this));

        RCLCPP_INFO(
            get_logger(),
            "[BSP_RC] activated: rate=%.1fHz, timeout=%.2fs, open-loop wrench + DLS allocation",
            rate,
            cmd_timeout_s_);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    {
        if (ctrl_timer_) {
            ctrl_timer_->cancel();
        }

        const bool was_enabled = disable_keyboard_output();
        if (was_enabled) {
            send_zero_thrust();
        }

        if (thruster_pub_) {
            thruster_pub_->on_deactivate();
        }

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
        if (ctrl_timer_) {
            ctrl_timer_->cancel();
        }

        const bool was_enabled = disable_keyboard_output();
        if (was_enabled) {
            send_zero_thrust();
        }

        return CallbackReturn::SUCCESS;
    }

private:
    bool load_parameters()
    {
        const double rate = this->get_parameter("control_rate_hz").as_double();
        cmd_timeout_s_ = this->get_parameter("cmd_timeout_s").as_double();

        surge_scale_ = this->get_parameter("surge_scale").as_double();
        sway_scale_  = this->get_parameter("sway_scale").as_double();
        heave_scale_ = this->get_parameter("heave_scale").as_double();
        yaw_scale_   = this->get_parameter("yaw_scale").as_double();

        alloc_lambda_   = this->get_parameter("alloc_lambda").as_double();
        deadzone_pct_   = this->get_parameter("deadzone_pct").as_double();
        thrust_max_pct_ = this->get_parameter("thrust_max_pct").as_double();
        thrust_min_pct_ = this->get_parameter("thrust_min_pct").as_double();

        alloc_vec_ = this->get_parameter("alloc_matrix").as_double_array();
        const auto thrust_limits =
            this->get_parameter("thrust_limits").as_double_array();

        const int64_t mode_pid_enable =
            this->get_parameter("mode_pid_enable_value").as_int();
        const int64_t mode_pid_disable =
            this->get_parameter("mode_pid_disable_value").as_int();
        const int64_t mode_keyboard_enable =
            this->get_parameter("mode_keyboard_enable_value").as_int();
        const int64_t mode_keyboard_disable =
            this->get_parameter("mode_keyboard_disable_value").as_int();

        const std::array<int64_t, 4> mode_values = {
            mode_pid_enable,
            mode_pid_disable,
            mode_keyboard_enable,
            mode_keyboard_disable
        };

        const bool mode_range_valid = std::all_of(
            mode_values.begin(),
            mode_values.end(),
            [](int64_t value) {
                return value >= 0 && value <= 255;
            });

        std::array<int64_t, 4> sorted_modes = mode_values;
        std::sort(sorted_modes.begin(), sorted_modes.end());
        const bool mode_unique =
            std::adjacent_find(sorted_modes.begin(), sorted_modes.end()) ==
            sorted_modes.end();

        if (!mode_range_valid || !mode_unique) {
            RCLCPP_ERROR(
                get_logger(),
                "[BSP_RC] mode command values must be unique and in [0,255]");
            return false;
        }

        pid_enable_value_ = static_cast<uint8_t>(mode_pid_enable);
        pid_disable_value_ = static_cast<uint8_t>(mode_pid_disable);
        keyboard_enable_value_ = static_cast<uint8_t>(mode_keyboard_enable);
        keyboard_disable_value_ = static_cast<uint8_t>(mode_keyboard_disable);

        const auto finite_nonnegative = [](double v) {
            return std::isfinite(v) && v >= 0.0;
        };

        if (!std::isfinite(rate) || rate <= 0.0 || rate > 1000.0 ||
            !std::isfinite(cmd_timeout_s_) || cmd_timeout_s_ <= 0.0 ||
            !finite_nonnegative(surge_scale_) ||
            !finite_nonnegative(sway_scale_) ||
            !finite_nonnegative(heave_scale_) ||
            !finite_nonnegative(yaw_scale_) ||
            !std::isfinite(alloc_lambda_) || alloc_lambda_ <= 0.0 ||
            alloc_lambda_ > 1000.0 ||
            !std::isfinite(deadzone_pct_) || deadzone_pct_ < 0.0 ||
            deadzone_pct_ > 100.0 ||
            !std::isfinite(thrust_min_pct_) ||
            !std::isfinite(thrust_max_pct_) ||
            thrust_min_pct_ < -100.0 ||
            thrust_max_pct_ > 100.0 ||
            thrust_min_pct_ > thrust_max_pct_) {
            RCLCPP_ERROR(get_logger(), "[BSP_RC] invalid control parameter");
            return false;
        }

        if (alloc_vec_.size() != 36U ||
            !std::all_of(
                alloc_vec_.begin(),
                alloc_vec_.end(),
                [](double v) { return std::isfinite(v); })) {
            RCLCPP_ERROR(
                get_logger(),
                "[BSP_RC] alloc_matrix must contain exactly 36 finite elements");
            return false;
        }

        if (thrust_limits.size() != 6U ||
            !std::all_of(
                thrust_limits.begin(),
                thrust_limits.end(),
                [](double v) {
                    return std::isfinite(v) && v > 0.0 && v <= 1e6;
                })) {
            RCLCPP_ERROR(
                get_logger(),
                "[BSP_RC] thrust_limits must contain 6 values in (0,1e6]");
            return false;
        }

        for (size_t i = 0; i < 6U; ++i) {
            thrust_limits_[i] = thrust_limits[i];
        }

        control_period_s_ = 1.0 / rate;
        return true;
    }

    void mode_callback(const hal::msg::HalModeControl::SharedPtr msg)
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) {
            return;
        }

        const uint8_t cmd = msg->modecontrol_cmd;

        // 3 = 开启键盘/遥控
        if (cmd == keyboard_enable_value_) {
            bool changed = false;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                if (!remote_enabled_) {
                    remote_enabled_ = true;
                    latest_cmd_ = RemoteCmd{};
                    last_cmd_time_ = std::chrono::steady_clock::now();
                    changed = true;
                }
            }

            if (changed) {
                watchdog_tripped_ = false;
                RCLCPP_INFO(
                    get_logger(),
                    "[BSP_RC] keyboard/remote enabled");
            }
            return;
        }

        // 4 = 关闭遥控；1 = PID 接管，也必须立即退出遥控
        if (cmd == keyboard_disable_value_ || cmd == pid_enable_value_) {
            const bool was_enabled = disable_keyboard_output();
            if (was_enabled) {
                send_zero_thrust();
                RCLCPP_INFO(
                    get_logger(),
                    "[BSP_RC] keyboard/remote disabled%s",
                    cmd == pid_enable_value_ ? " (PID takeover)" : "");
            }
            return;
        }

        // 2 = 关闭 PID，与遥控节点当前状态无关
        if (cmd == pid_disable_value_) {
            return;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "[BSP_RC] unknown mode command: %u",
            static_cast<unsigned>(cmd));
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

        watchdog_tripped_ = false;
        return was_enabled;
    }

    void remote_callback(const hal::msg::HalRemoteControl::SharedPtr msg)
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) {
            return;
        }

        const std::array<double, 6> channels = {
            static_cast<double>(msg->tunnel1_para),
            static_cast<double>(msg->tunnel2_para),
            static_cast<double>(msg->tunnel3_para),
            static_cast<double>(msg->tunnel4_para),
            static_cast<double>(msg->tunnel5_para),
            static_cast<double>(msg->tunnel6_para)
        };

        if (!std::all_of(
                channels.begin(),
                channels.end(),
                [](double value) { return std::isfinite(value); })) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "[BSP_RC] /hal/remotecontrol contains non-finite channel");
            return;
        }

        std::lock_guard<std::mutex> lock(cmd_mutex_);
        if (!remote_enabled_) {
            return;
        }

        latest_cmd_.surge = normalize_remote_channel(channels[0]);
        latest_cmd_.sway  = normalize_remote_channel(channels[1]);
        latest_cmd_.heave = normalize_remote_channel(channels[2]);
        latest_cmd_.yaw   = normalize_remote_channel(channels[3]);
        latest_cmd_.fresh = true;

        // tunnel5 / tunnel6 不再承担定深功能，目前保留不用。
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

        // 未进入遥控模式时完全不发布，避免与 PID 节点争用 /hal/thruster/cmd
        if (!enabled) {
            return;
        }

        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_cmd_time).count();

        if (!cmd.fresh || age > cmd_timeout_s_) {
            if (cmd.fresh && !watchdog_tripped_) {
                RCLCPP_WARN(
                    get_logger(),
                    "[BSP_RC] command watchdog triggered, age=%.3fs",
                    age);
                watchdog_tripped_ = true;
            }

            send_zero_thrust();
            return;
        }

        watchdog_tripped_ = false;

        // ---------------------------------------------------------------------
        // 遥控量 -> 体坐标系期望广义力/力矩
        // 只做开环，不读取 IMU/DVL/深度计，不做 PID/FF/DOB。
        // ---------------------------------------------------------------------
        const std::array<double, 6> tau_cmd = {
            cmd.surge * surge_scale_,  // Fx
            cmd.sway  * sway_scale_,   // Fy
            cmd.heave * heave_scale_,  // Fz
            0.0,                       // Mx
            0.0,                       // My
            cmd.yaw * yaw_scale_       // Mz
        };

        // 与 motioncontrol 一致：逐次截断阻尼最小二乘分配
        const std::array<double, 6> u_cmd = allocate_thrust(tau_cmd);

        auto out_msg = std_msgs::msg::Float64MultiArray();
        out_msg.data.reserve(6);

        for (size_t k = 0; k < 6U; ++k) {
            const double u_eff =
                (std::abs(u_cmd[k]) <
                 deadzone_pct_ * 0.01 * thrust_limits_[k])
                    ? 0.0
                    : u_cmd[k];

            const double pct = std::clamp(
                u_eff / thrust_limits_[k] * 100.0,
                thrust_min_pct_,
                thrust_max_pct_);

            out_msg.data.push_back(pct);
        }

        thruster_pub_->publish(out_msg);
    }

    void send_zero_thrust()
    {
        if (!thruster_pub_ || !thruster_pub_->is_activated()) {
            return;
        }

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        thruster_pub_->publish(msg);
    }

    // -------------------------------------------------------------------------
    // 控制分配：与 bsp_motioncontrol_node 相同的逐次截断 DLS
    // 约束 |u_k| <= thrust_limits_[k]
    // -------------------------------------------------------------------------
    std::array<double, 6> allocate_thrust(
        const std::array<double, 6> & tau)
    {
        std::array<double, 6> u{};
        std::array<bool, 6> pinned{};

        const double lambda =
            (std::isfinite(alloc_lambda_) && alloc_lambda_ > 1e-9)
                ? alloc_lambda_
                : 0.01;

        for (int iter = 0; iter < 6; ++iter) {
            int n_free = 0;
            int free_idx[6];

            for (int k = 0; k < 6; ++k) {
                if (!pinned[k]) {
                    free_idx[n_free++] = k;
                }
            }

            if (n_free == 0) {
                break;
            }

            // 已饱和推进器的贡献从目标中扣除
            std::array<double, 6> tau_rem = tau;
            for (int k = 0; k < 6; ++k) {
                if (!pinned[k]) {
                    continue;
                }

                for (int i = 0; i < 6; ++i) {
                    tau_rem[i] -= alloc_vec_[i * 6 + k] * u[k];
                }
            }

            // M = T_f*T_f^T + lambda^2*I
            double M[6][6] = {};
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    double sum = 0.0;

                    for (int n = 0; n < n_free; ++n) {
                        const int k = free_idx[n];
                        sum +=
                            alloc_vec_[i * 6 + k] *
                            alloc_vec_[j * 6 + k];
                    }

                    M[i][j] = sum;
                    if (i == j) {
                        M[i][j] += lambda * lambda;
                    }
                }
            }

            // Cholesky: M = L*L^T
            double L[6][6] = {};
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j <= i; ++j) {
                    double sum = M[i][j];

                    for (int k = 0; k < j; ++k) {
                        sum -= L[i][k] * L[j][k];
                    }

                    if (i == j) {
                        L[i][j] = std::sqrt(std::max(sum, 1e-12));
                    } else {
                        L[i][j] = sum / L[j][j];
                    }
                }
            }

            // 解 M*y = tau_rem
            double y[6] = {};
            double z[6] = {};

            for (int i = 0; i < 6; ++i) {
                double sum = tau_rem[i];
                for (int j = 0; j < i; ++j) {
                    sum -= L[i][j] * z[j];
                }
                z[i] = sum / L[i][i];
            }

            for (int i = 5; i >= 0; --i) {
                double sum = z[i];
                for (int j = i + 1; j < 6; ++j) {
                    sum -= L[j][i] * y[j];
                }
                y[i] = sum / L[i][i];
            }

            // u_f = T_f^T * y，并检查饱和
            bool any_sat = false;

            for (int j = 0; j < 6; ++j) {
                if (pinned[j]) {
                    continue;
                }

                double value = 0.0;
                for (int i = 0; i < 6; ++i) {
                    value += alloc_vec_[i * 6 + j] * y[i];
                }

                const double limit = thrust_limits_[j];

                if (value > limit) {
                    u[j] = limit;
                    pinned[j] = true;
                    any_sat = true;
                } else if (value < -limit) {
                    u[j] = -limit;
                    pinned[j] = true;
                    any_sat = true;
                } else {
                    u[j] = value;
                }
            }

            if (!any_sat) {
                break;
            }
        }

        for (int k = 0; k < 6; ++k) {
            u[k] = std::clamp(
                u[k],
                -thrust_limits_[k],
                thrust_limits_[k]);
        }

        return u;
    }

private:
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

    double cmd_timeout_s_ = 1.0;
    double control_period_s_ = 0.02;

    // 遥控独立开环比例
    double surge_scale_ = 300.0;
    double sway_scale_  = 160.0;
    double heave_scale_ = 80.0;
    double yaw_scale_   = 50.0;

    // 与 motioncontrol 统一的执行器参数
    double alloc_lambda_ = 0.01;
    double deadzone_pct_ = 3.0;
    double thrust_max_pct_ = 100.0;
    double thrust_min_pct_ = -100.0;

    std::array<double, 6> thrust_limits_{
        441.0, 69.0, 69.0, 69.0, 69.0, 69.0};

    std::vector<double> alloc_vec_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<BspRemoteControlNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
