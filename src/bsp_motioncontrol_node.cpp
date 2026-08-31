/**
 * @file bsp_motioncontrol_node.cpp
 * @brief BSP层运动控制节点 —— 6-DOF 动力学前馈 + PID 反馈控制
 *
 * ## 输入 (Subscriptions)
 *   - /hal/modecontrol    (HalModeControl):     模式控制命令
 *         1=开启PID, 2=关闭PID, 3=开启键盘遥控, 4=关闭键盘遥控
 *   - /hal/remotecontrol  (HalRemoteControl):   遥控通道量, 353~1695, 1024 为中位
 *         其中 pitch / roll 当前保留, 内部强制置零
 *   - /hal/inertialnavi    (HalInertialnavi):   航行器姿态 (yaw/pitch/roll)
 *   - /hal/dvl             (HalDvl):            体坐标系速度 (vx/vy/vz)
 *   - /hal/depthsensor     (HalDepthsensor):     深度 (depth_avg)
 *   - /hal/mainthruster    (HalMainthruster):    主推状态反馈
 *   - /hal/auxithruster    (HalAuxithruster):    辅推状态反馈
 *   - /hal/tailservo       (HalTailservo):       尾舵状态反馈
 *   - /hal/wingservo       (HalWingservo):       翼舵状态反馈
 *
 * ## 输出 (Publishers)
 *   - /hal/thruster/cmd    (Float64MultiArray):  推进器指令
 *         data[0] = 主推百分比 (CAN ID 0x301)
 *         data[1..5] = 辅推电调 ID2..ID6 百分比
 *   - /hal/servo/tail_cmd  (Float64MultiArray):  [框架] 尾舵指令
 *   - /hal/servo/wing_cmd  (Float64MultiArray):  [框架] 翼舵指令
 *
 * ## 控制算法
 *   τ_des = τ_ff + τ_pid              (1) 期望合力/力矩 = 前馈 + 反馈
 *   u     = T⁺ · τ_des                (2) 控制分配: 伪逆映射到各推进器
 *
 *   前馈 τ_ff 补偿稳态水动力阻尼与浮力:
 *     F_x_ff = D_x_lin·vx + D_x_quad·vx·|vx|
 *     F_y_ff = D_y_lin·vy + D_y_quad·vy·|vy|
 *     F_z_ff = buoyancy_trim
 *     M_z_ff = D_yaw_lin·r + D_yaw_quad·r·|r|
 *
 *   反馈 τ_pid 为六通道离散PID, 带积分抗饱和与微分低通滤波:
 *     e[k] = x_des[k] - x_real[k]
 *     P[k] = Kp · e[k]
 *     I[k] = I[k-1] + Ki · e[k] · dt   (带限幅)
 *     D[k] = Kd · (e[k] - e[k-1]) / dt  (一阶低通滤波)
 *
 * @author BSP Motion Control Team
 * @date   2026-06-22
 */

 #include <algorithm>
 #include <array>
 #include <atomic>
 #include <chrono>
 #include <cstdint>
 #include <cmath>
 #include <exception>
 #include <memory>
 #include <mutex>
 #include <stdexcept>
 #include <string>
 #include <vector>
 
 #include "rclcpp/rclcpp.hpp"
 #include "rclcpp_lifecycle/lifecycle_node.hpp"
 #include "rclcpp_lifecycle/lifecycle_publisher.hpp"
 #include "lifecycle_msgs/msg/state.hpp"
 #include "std_msgs/msg/float64_multi_array.hpp"
 
 #include "hal/msg/hal_inertialnavi.hpp"
 #include "hal/msg/hal_dvl.hpp"
 #include "hal/msg/hal_depthsensor.hpp"
 #include "hal/msg/hal_mainthruster.hpp"
 #include "hal/msg/hal_auxithruster.hpp"
 #include "hal/msg/hal_tailservo.hpp"
 #include "hal/msg/hal_wingservo.hpp"
 #include "hal/msg/hal_mode_control.hpp"
 #include "hal/msg/hal_remote_control.hpp"
 
 using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
 
 // ============================================================================
 // 工具: 角度归一化到 [-PI, PI]
 // ============================================================================
 inline double wrap_angle(double angle) {
     while (angle >  M_PI) angle -= 2.0 * M_PI;
     while (angle < -M_PI) angle += 2.0 * M_PI;
     return angle;
 }

 static constexpr double REMOTE_CHANNEL_MIN = 353.0;
 static constexpr double REMOTE_CHANNEL_MID = 1024.0;
 static constexpr double REMOTE_CHANNEL_MAX = 1695.0;
 static constexpr double REMOTE_CHANNEL_HALF_RANGE =
     REMOTE_CHANNEL_MAX - REMOTE_CHANNEL_MID;
 static constexpr double REMOTE_CHANNEL_DEADBAND = 20.0;

 inline double normalize_remote_channel(double value) {
     const double clamped = std::clamp(value, REMOTE_CHANNEL_MIN, REMOTE_CHANNEL_MAX);
     if (std::abs(clamped - REMOTE_CHANNEL_MID) <= REMOTE_CHANNEL_DEADBAND) {
         return 0.0;
     }
     return std::clamp(
         (clamped - REMOTE_CHANNEL_MID) / REMOTE_CHANNEL_HALF_RANGE,
         -1.0,
         1.0);
 }

 inline double map_remote_depth(double value) {
     const double clamped = std::clamp(value, REMOTE_CHANNEL_MIN, REMOTE_CHANNEL_MAX);
     return (clamped - REMOTE_CHANNEL_MIN) /
         (REMOTE_CHANNEL_MAX - REMOTE_CHANNEL_MIN) * 10.0;
 }
 
 // ============================================================================
 // 离散PID控制器 (带积分抗饱和 + 微分低通滤波)
 // ============================================================================
 struct PidController {
     double kp = 0.0;
     double ki = 0.0;
     double kd = 0.0;
     double max_i = 0.0;       // 积分限幅 (绝对值)
     double max_out = 0.0;     // 输出限幅 (绝对值)
     double alpha = 0.0;       // 微分滤波系数 [0,1), 越大滤波越强
 
     double integral = 0.0;
     double prev_error = 0.0;
     double prev_filtered_deriv = 0.0;
 
     /**
      * @brief 执行一步PID更新
      * @param error  当前误差 (setpoint - measurement)
      * @param dt     控制周期 (秒)
      * @return       控制输出 (已限幅)
      */
     double update(double error, double dt) {
         if (dt <= 0.0) return 0.0;
 
         // ---- P项 ----
         double p_term = kp * error;
 
         // ---- I项 (梯形积分 + 抗饱和钳位) ----
         double i_term = 0.0;
         if (ki > 0.0 && max_i > 0.0) {
             integral += ki * error * dt;
             integral  = std::clamp(integral, -max_i, max_i);
             i_term    = integral;
         }
 
         // ---- D项 (一阶低通滤波微分) ----
         double d_term = 0.0;
         if (kd > 0.0) {
             double raw_deriv = (error - prev_error) / dt;
             double filtered  = alpha * prev_filtered_deriv + (1.0 - alpha) * raw_deriv;
             prev_filtered_deriv = filtered;
             d_term = kd * filtered;
         }
         prev_error = error;
 
         // ---- 合成 + 输出限幅 ----
         double out = p_term + i_term + d_term;
         if (max_out > 0.0) {
             out = std::clamp(out, -max_out, max_out);
         }
 
         return out;
     }
 
     void reset() {
         integral            = 0.0;
         prev_error          = 0.0;
         prev_filtered_deriv = 0.0;
     }
 };
 
 // ============================================================================
 // 六自由度目标指令
 // ============================================================================
 struct TargetSetpoint {
     double vx    = 0.0;   // m/s  纵荡速度
     double vy    = 0.0;   // m/s  横荡速度
     double depth = 0.0;   // m    深度 (正=向下)
     double yaw   = 0.0;   // rad  艏向角
     double pitch = 0.0;   // rad  纵倾 (强制锁定为0)
     double roll  = 0.0;   // rad  横摇 (强制锁定为0)
     bool   valid = false; // 是否收到过有效指令
 };
 
 // ============================================================================
 // 航行器状态 (聚合各传感器)
 // ============================================================================
 struct VehicleState {
     // -- 姿态 (来自 IMU) --
     double yaw   = 0.0;
     double pitch = 0.0;
     double roll  = 0.0;
     bool   imu_valid = false;
 
     // -- 体坐标系线速度 (来自 DVL) --
     double vx = 0.0;
     double vy = 0.0;
     double vz = 0.0;
     bool   dvl_valid = false;
 
     // -- 深度 (来自水深传感器) --
     double depth     = 0.0;
     bool   depth_valid = false;
 
     // -- 推进器状态 --
     double main_thrust_pct   = 0.0;
     bool   main_fault        = false;
     std::array<double, 5> aux_thrust_pct{};
     std::array<bool, 5>   aux_fault{};
 
     // -- 舵机状态 --
     std::array<double, 4> tail_position{};
     std::array<double, 2> wing_position{};
 };
 
 // ============================================================================
 // 合力/力矩 (体坐标系, 6-DOF wrench)
 // ============================================================================
 struct Wrench {
     double Fx = 0.0;  // N  纵荡力
     double Fy = 0.0;  // N  横荡力
     double Fz = 0.0;  // N  垂荡力 (正=向下)
     double Mx = 0.0;  // Nm 横摇力矩
     double My = 0.0;  // Nm 纵倾力矩
     double Mz = 0.0;  // Nm 转艏力矩
 };
 
 // ============================================================================
 // 主节点类
 // ============================================================================
 class BspMotionControlNode : public rclcpp_lifecycle::LifecycleNode {
 public:
     explicit BspMotionControlNode(const std::string & node_name,
                                   bool intra_process_comms = false)
         : rclcpp_lifecycle::LifecycleNode(node_name,
               rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
     {
         // ----- 控制周期参数 -----
         this->declare_parameter<double>("control_rate_hz", 50.0);
 
         // ----- PID 参数 (每自由度独立) -----
         // Surge (vx)
         this->declare_parameter<double>("pid_vx.kp", 200.0);
         this->declare_parameter<double>("pid_vx.ki", 20.0);
         this->declare_parameter<double>("pid_vx.kd", 10.0);
         this->declare_parameter<double>("pid_vx.max_i", 50.0);
         this->declare_parameter<double>("pid_vx.max_out", 300.0);
 
         // Sway (vy)
         this->declare_parameter<double>("pid_vy.kp", 200.0);
         this->declare_parameter<double>("pid_vy.ki", 20.0);
         this->declare_parameter<double>("pid_vy.kd", 10.0);
         this->declare_parameter<double>("pid_vy.max_i", 50.0);
         this->declare_parameter<double>("pid_vy.max_out", 300.0);
 
         // Depth
         this->declare_parameter<double>("pid_depth.kp", 300.0);
         this->declare_parameter<double>("pid_depth.ki", 30.0);
         this->declare_parameter<double>("pid_depth.kd", 50.0);
         this->declare_parameter<double>("pid_depth.max_i", 80.0);
         this->declare_parameter<double>("pid_depth.max_out", 400.0);
 
         // Yaw
         this->declare_parameter<double>("pid_yaw.kp", 400.0);
         this->declare_parameter<double>("pid_yaw.ki", 10.0);
         this->declare_parameter<double>("pid_yaw.kd", 80.0);
         this->declare_parameter<double>("pid_yaw.max_i", 40.0);
         this->declare_parameter<double>("pid_yaw.max_out", 500.0);
 
         // Pitch
         this->declare_parameter<double>("pid_pitch.kp", 150.0);
         this->declare_parameter<double>("pid_pitch.ki", 5.0);
         this->declare_parameter<double>("pid_pitch.kd", 30.0);
         this->declare_parameter<double>("pid_pitch.max_i", 20.0);
         this->declare_parameter<double>("pid_pitch.max_out", 150.0);
 
         // Roll
         this->declare_parameter<double>("pid_roll.kp", 150.0);
         this->declare_parameter<double>("pid_roll.ki", 5.0);
         this->declare_parameter<double>("pid_roll.kd", 30.0);
         this->declare_parameter<double>("pid_roll.max_i", 20.0);
         this->declare_parameter<double>("pid_roll.max_out", 150.0);
 
         // ----- 前馈参数 (阻尼系数) -----
         this->declare_parameter<double>("ff.drag_linear_x",   50.0);
         this->declare_parameter<double>("ff.drag_quadratic_x", 120.0);
         this->declare_parameter<double>("ff.drag_linear_y",   60.0);
         this->declare_parameter<double>("ff.drag_quadratic_y", 140.0);
         this->declare_parameter<double>("ff.drag_linear_z",   80.0);
         this->declare_parameter<double>("ff.drag_quadratic_z", 180.0);
         this->declare_parameter<double>("ff.drag_linear_yaw",   30.0);
         this->declare_parameter<double>("ff.drag_quadratic_yaw", 80.0);
         this->declare_parameter<double>("ff.buoyancy_trim",   0.0);
 
         // ----- 控制分配矩阵 T (6×6, 行优先) -----
         //  T ∈ R^(6×6):   τ = T · u
         //  列/输出顺序: [主推0x301, 辅推ID2, 辅推ID3, 辅推ID4, 辅推ID5, 辅推ID6]
         //  行顺序: [Fx, Fy, Fz, Mx, My, Mz]
         std::vector<double> default_T = {
             // Main  ID2   ID3   ID4   ID5   ID6
                1.0,  0.0,  0.0,  0.0,  0.5,  0.5,  // Fx
                0.0,  0.0,  0.0,  1.0,  1.0,  0.0,  // Fy
                0.0,  1.0,  1.0,  0.0,  0.0,  0.0,  // Fz
                0.0,  0.1, -0.1,  0.0,  0.0,  0.0,  // Mx
                0.0,  0.3,  0.3,  0.0,  0.0,  0.0,  // My
                0.0,  0.0,  0.0,  0.2, -0.2,  0.1,  // Mz
         };
         this->declare_parameter<std::vector<double>>("alloc_matrix", default_T);
 
         // ----- 执行器限幅 -----
         this->declare_parameter<double>("thrust_max_pct",  100.0);
         this->declare_parameter<double>("thrust_min_pct", -100.0);
         this->declare_parameter<double>("servo_max_deg",    45.0);
 
         // ----- 控制使能开关 (方便调试各通道) -----
         this->declare_parameter<bool>("enable_surge",  true);
         this->declare_parameter<bool>("enable_sway",   true);
         this->declare_parameter<bool>("enable_depth",  true);
         this->declare_parameter<bool>("enable_yaw",    true);
         this->declare_parameter<bool>("enable_pitch",  true);
         this->declare_parameter<bool>("enable_roll",   true);
         this->declare_parameter<bool>("enable_ff",     true);
         this->declare_parameter<bool>("enable_pid",    true);
         this->declare_parameter<bool>("enable_servo",  false);  // 舵机暂不使能
 
         // ----- 看门狗超时 (秒) -----
         this->declare_parameter<double>("cmd_timeout_s", 1.0);
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
     }
 
     // ========================================================================
     // 生命周期: on_configure
     // ========================================================================
     CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
         RCLCPP_INFO(get_logger(), "[MC] on_configure —— 创建订阅/发布/定时器");
 
         try {
             load_parameters();
         } catch (const std::exception &exception) {
             RCLCPP_ERROR(get_logger(), "[MC] invalid configuration: %s", exception.what());
             return CallbackReturn::FAILURE;
         }
 
         auto qos_sensor = rclcpp::QoS(10).best_effort();
         auto qos_cmd    = rclcpp::QoS(10).reliable();
 
         // -- 传感器订阅 --
         imu_sub_   = this->create_subscription<hal::msg::HalInertialnavi>(
             "/hal/inertialnavi", qos_sensor,
             std::bind(&BspMotionControlNode::imu_cb, this, std::placeholders::_1));
         dvl_sub_   = this->create_subscription<hal::msg::HalDvl>(
             "/hal/dvl", qos_sensor,
             std::bind(&BspMotionControlNode::dvl_cb, this, std::placeholders::_1));
         depth_sub_ = this->create_subscription<hal::msg::HalDepthsensor>(
             "/hal/depthsensor", qos_sensor,
             std::bind(&BspMotionControlNode::depth_cb, this, std::placeholders::_1));
 
         // -- 推进器状态订阅 --
         main_thruster_sub_ = this->create_subscription<hal::msg::HalMainthruster>(
             "/hal/mainthruster", qos_sensor,
             std::bind(&BspMotionControlNode::main_thruster_cb, this, std::placeholders::_1));
         aux_thruster_sub_  = this->create_subscription<hal::msg::HalAuxithruster>(
             "/hal/auxithruster", qos_sensor,
             std::bind(&BspMotionControlNode::aux_thruster_cb, this, std::placeholders::_1));
 
         // -- 舵机状态订阅 --
         tail_servo_sub_ = this->create_subscription<hal::msg::HalTailservo>(
             "/hal/tailservo", qos_sensor,
             std::bind(&BspMotionControlNode::tail_servo_cb, this, std::placeholders::_1));
         wing_servo_sub_ = this->create_subscription<hal::msg::HalWingservo>(
             "/hal/wingservo", qos_sensor,
             std::bind(&BspMotionControlNode::wing_servo_cb, this, std::placeholders::_1));
 
         // -- 模式与遥控通道订阅 (来自统一 UDP 接收节点) --
         mode_sub_ = this->create_subscription<hal::msg::HalModeControl>(
             this->get_parameter("mode_topic").as_string(), qos_cmd,
             std::bind(&BspMotionControlNode::mode_cb, this, std::placeholders::_1));
         remote_sub_ = this->create_subscription<hal::msg::HalRemoteControl>(
             this->get_parameter("remote_topic").as_string(), qos_cmd,
             std::bind(&BspMotionControlNode::remote_cb, this, std::placeholders::_1));
 
         // -- 推进器指令发布 --
         thruster_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
             "/hal/thruster/cmd", 10);
 
         // -- [框架] 舵机指令发布 (当前不使能) --
         tail_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
             "/hal/servo/tail_cmd", 10);
         wing_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
             "/hal/servo/wing_cmd", 10);
 
         // -- 控制定时器 --
         const double rate = 1.0 / control_period_s_;
         const int period_ms = static_cast<int>(1000.0 / rate);
         control_timer_ = this->create_wall_timer(
             std::chrono::milliseconds(period_ms),
             std::bind(&BspMotionControlNode::control_loop, this));
 
         RCLCPP_INFO(get_logger(), "[MC] 配置完成, 控制频率 %.1f Hz", rate);
         return CallbackReturn::SUCCESS;
     }
 
     // ========================================================================
     // 生命周期: on_activate
     // ========================================================================
     CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override {
         RCLCPP_INFO(get_logger(), "[MC] on_activate —— 加载参数, 复位PID");
 
         thruster_cmd_pub_->on_activate();
         tail_cmd_pub_->on_activate();
         wing_cmd_pub_->on_activate();
 
         try {
             load_parameters();
             reset_all_pid();
             clear_target_state();
             clear_vehicle_state();
             active_ = true;
             LifecycleNode::on_activate(state);
             return CallbackReturn::SUCCESS;
         } catch (const std::exception &exception) {
             RCLCPP_ERROR(get_logger(), "[MC] activation failed: %s", exception.what());
             active_ = false;
             clear_target_state();
             clear_vehicle_state();
             reset_all_pid();
             thruster_cmd_pub_->on_deactivate();
             tail_cmd_pub_->on_deactivate();
             wing_cmd_pub_->on_deactivate();
             return CallbackReturn::FAILURE;
         }
     }
 
     // ========================================================================
     // 生命周期: on_deactivate
     // ========================================================================
     CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override {
         RCLCPP_INFO(get_logger(), "[MC] on_deactivate —— 停机");

         const bool was_enabled = disable_pid_output();
         active_ = false;
         clear_vehicle_state();
         if (was_enabled) {
             send_zero_thrust();
         } else {
             reset_all_pid();
         }
         thruster_cmd_pub_->on_deactivate();
         tail_cmd_pub_->on_deactivate();
         wing_cmd_pub_->on_deactivate();

         LifecycleNode::on_deactivate(state);
         return CallbackReturn::SUCCESS;
     }
 
     // ========================================================================
     // 生命周期: on_cleanup
     // ========================================================================
     CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
         RCLCPP_INFO(get_logger(), "[MC] on_cleanup");
 
         imu_sub_.reset();   dvl_sub_.reset();   depth_sub_.reset();
         main_thruster_sub_.reset();  aux_thruster_sub_.reset();
         tail_servo_sub_.reset();     wing_servo_sub_.reset();
         mode_sub_.reset();
         remote_sub_.reset();
         thruster_cmd_pub_.reset();
         tail_cmd_pub_.reset();       wing_cmd_pub_.reset();
         control_timer_.reset();
 
         return CallbackReturn::SUCCESS;
     }
 
     // ========================================================================
     // 生命周期: on_shutdown
     // ========================================================================
     CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override {
         const bool was_enabled = disable_pid_output();
         active_ = false;
         clear_vehicle_state();
         if (was_enabled) {
             send_zero_thrust();
         }
         return CallbackReturn::SUCCESS;
     }

     CallbackReturn on_error(const rclcpp_lifecycle::State &) override {
         const bool was_enabled = disable_pid_output();
         active_ = false;
         clear_vehicle_state();
         if (was_enabled) {
             send_zero_thrust();
         }
         return CallbackReturn::SUCCESS;
     }
 
 private:
     // ========================================================================
     // 参数加载
     // ========================================================================
     void load_parameters() {
         // PID
         auto load_pid = [this](const std::string & prefix, PidController & pid) {
             pid.kp      = this->get_parameter(prefix + ".kp").as_double();
             pid.ki      = this->get_parameter(prefix + ".ki").as_double();
             pid.kd      = this->get_parameter(prefix + ".kd").as_double();
             pid.max_i   = this->get_parameter(prefix + ".max_i").as_double();
             pid.max_out = this->get_parameter(prefix + ".max_out").as_double();
             pid.alpha   = 0.7;  // 固定微分滤波系数
             if (!std::isfinite(pid.kp) || !std::isfinite(pid.ki) ||
                 !std::isfinite(pid.kd) || !std::isfinite(pid.max_i) ||
                 !std::isfinite(pid.max_out) || pid.max_i < 0.0 ||
                 pid.max_out <= 0.0) {
                 throw std::invalid_argument(
                     prefix + " PID gains/limits must be finite with max_i >= 0 and max_out > 0");
             }
         };
         load_pid("pid_vx",    pid_vx_);
         load_pid("pid_vy",    pid_vy_);
         load_pid("pid_depth", pid_depth_);
         load_pid("pid_yaw",   pid_yaw_);
         load_pid("pid_pitch", pid_pitch_);
         load_pid("pid_roll",  pid_roll_);
 
         // Feedforward
         ff_drag_lin_x_    = this->get_parameter("ff.drag_linear_x").as_double();
         ff_drag_quad_x_   = this->get_parameter("ff.drag_quadratic_x").as_double();
         ff_drag_lin_y_    = this->get_parameter("ff.drag_linear_y").as_double();
         ff_drag_quad_y_   = this->get_parameter("ff.drag_quadratic_y").as_double();
         ff_drag_lin_z_    = this->get_parameter("ff.drag_linear_z").as_double();
         ff_drag_quad_z_   = this->get_parameter("ff.drag_quadratic_z").as_double();
         ff_drag_lin_yaw_  = this->get_parameter("ff.drag_linear_yaw").as_double();
         ff_drag_quad_yaw_ = this->get_parameter("ff.drag_quadratic_yaw").as_double();
         ff_buoyancy_      = this->get_parameter("ff.buoyancy_trim").as_double();
         for (const double value : {
                 ff_drag_lin_x_, ff_drag_quad_x_, ff_drag_lin_y_, ff_drag_quad_y_,
                 ff_drag_lin_z_, ff_drag_quad_z_, ff_drag_lin_yaw_,
                 ff_drag_quad_yaw_, ff_buoyancy_}) {
             if (!std::isfinite(value)) {
                 throw std::invalid_argument("feedforward parameters must be finite");
             }
         }
 
         // Allocation matrix
         alloc_vec_ = this->get_parameter("alloc_matrix").as_double_array();
         constexpr size_t EXPECTED_ALLOC_SIZE = 36;  // 6×6
         if (alloc_vec_.size() != EXPECTED_ALLOC_SIZE) {
             throw std::invalid_argument("alloc_matrix must contain exactly 36 elements");
         }
         if (!std::all_of(alloc_vec_.begin(), alloc_vec_.end(),
                 [](double value) { return std::isfinite(value); })) {
             throw std::invalid_argument("alloc_matrix elements must be finite");
         }
 
         // Limits
         thrust_max_pct_ = this->get_parameter("thrust_max_pct").as_double();
         thrust_min_pct_ = this->get_parameter("thrust_min_pct").as_double();
         if (!std::isfinite(thrust_min_pct_) || !std::isfinite(thrust_max_pct_) ||
             thrust_min_pct_ < -100.0 || thrust_max_pct_ > 100.0 ||
             thrust_min_pct_ > thrust_max_pct_) {
             throw std::invalid_argument(
                 "thrust limits must be finite, ordered, and within [-100, 100]");
         }
 
         // Enable flags
         en_surge_ = this->get_parameter("enable_surge").as_bool();
         en_sway_  = this->get_parameter("enable_sway").as_bool();
         en_depth_ = this->get_parameter("enable_depth").as_bool();
         en_yaw_   = this->get_parameter("enable_yaw").as_bool();
         en_pitch_ = this->get_parameter("enable_pitch").as_bool();
         en_roll_  = this->get_parameter("enable_roll").as_bool();
         en_ff_    = this->get_parameter("enable_ff").as_bool();
         en_pid_   = this->get_parameter("enable_pid").as_bool();
         en_servo_ = this->get_parameter("enable_servo").as_bool();
 
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
             throw std::invalid_argument(
                 "mode command values must be unique and in [0, 255]");
         }

         pid_enable_value_ = static_cast<uint8_t>(mode_pid_enable);
         pid_disable_value_ = static_cast<uint8_t>(mode_pid_disable);
         keyboard_enable_value_ = static_cast<uint8_t>(mode_keyboard_enable);
         keyboard_disable_value_ = static_cast<uint8_t>(mode_keyboard_disable);

         const double control_rate_hz =
             this->get_parameter("control_rate_hz").as_double();
         if (!std::isfinite(cmd_timeout_s_) || cmd_timeout_s_ <= 0.0 ||
             !std::isfinite(control_rate_hz) || control_rate_hz <= 0.0 ||
             control_rate_hz > 1000.0) {
             throw std::invalid_argument(
                 "cmd_timeout_s must be positive and control_rate_hz must be in (0, 1000]");
         }
         control_period_s_ = 1.0 / control_rate_hz;
 
         RCLCPP_INFO(get_logger(),
             "[MC] 参数加载完成: PID/FF/Alloc 已就绪 | "
             "使能: surge=%d sway=%d depth=%d yaw=%d pitch=%d roll=%d ff=%d pid=%d servo=%d",
             en_surge_, en_sway_, en_depth_, en_yaw_, en_pitch_, en_roll_,
             en_ff_, en_pid_, en_servo_);
     }
 
     void reset_all_pid() {
         pid_vx_.reset();   pid_vy_.reset();
         pid_depth_.reset(); pid_yaw_.reset();
         pid_pitch_.reset(); pid_roll_.reset();
     }
 
     // ========================================================================
     // 传感器回调 (线程安全, 使用 std::atomic / mutex)
     // ========================================================================
     void imu_cb(const hal::msg::HalInertialnavi::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_.yaw       = static_cast<double>(msg->yaw);
         state_.pitch     = static_cast<double>(msg->pitch);
         state_.roll      = static_cast<double>(msg->roll);
         state_.imu_valid = (msg->connection_status == 1);
     }
 
     void dvl_cb(const hal::msg::HalDvl::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_.vx        = static_cast<double>(msg->velocity_x);
         state_.vy        = static_cast<double>(msg->velocity_y);
         state_.vz        = static_cast<double>(msg->velocity_z);
         state_.dvl_valid = (msg->connection_status == 1);
     }
 
     void depth_cb(const hal::msg::HalDepthsensor::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_.depth       = static_cast<double>(msg->depth_avg);
         state_.depth_valid = (msg->connection_status == 1);
     }
 
     void main_thruster_cb(const hal::msg::HalMainthruster::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_.main_fault = (msg->fault_status != 0);
     }
 
     void aux_thruster_cb(const hal::msg::HalAuxithruster::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         for (size_t i = 0; i < 5; ++i) {
             state_.aux_fault[i] = (msg->fault_status[i] != 0);
         }
     }
 
     void tail_servo_cb(const hal::msg::HalTailservo::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         for (size_t i = 0; i < 4; ++i) {
             state_.tail_position[i] = static_cast<double>(msg->position[i]);
         }
     }
 
     void wing_servo_cb(const hal::msg::HalWingservo::SharedPtr msg) {
         if (!active_) return;
         std::lock_guard<std::mutex> lock(state_mutex_);
         for (size_t i = 0; i < 2; ++i) {
             state_.wing_position[i] = static_cast<double>(msg->position[i]);
         }
     }
 
     // ========================================================================
     // 模式与遥控通道回调
     // ========================================================================
     void mode_cb(const hal::msg::HalModeControl::SharedPtr msg) {
         if (!active_) return;

         const uint8_t cmd = msg->modecontrol_cmd;

         // 1: 开启 PID 控制。
         if (cmd == pid_enable_value_) {
             bool mode_changed = false;
             {
                 std::lock_guard<std::mutex> lock(cmd_mutex_);
                 if (!pid_mode_enabled_) {
                     pid_mode_enabled_ = true;
                     target_ = TargetSetpoint{};
                     last_cmd_time_ = std::chrono::steady_clock::now();
                     mode_changed = true;
                 }
             }
             if (mode_changed) {
                 reset_all_pid();
                 yaw_target_initialized_ = false;
                 RCLCPP_INFO(get_logger(),
                     "[MC] PID mode enabled (cmd=%u)",
                     static_cast<unsigned>(cmd));
             }
             return;
         }

         // 2: 显式关闭 PID。
         // 3: 开启键盘遥控，键盘优先接管，因此 PID 节点必须自动退出。
         if (cmd == pid_disable_value_ || cmd == keyboard_enable_value_) {
             const bool was_enabled = disable_pid_output();
             if (was_enabled) {
                 send_zero_thrust();  // 仅在退出瞬间发布一次零推力
                 RCLCPP_INFO(get_logger(),
                     "[MC] PID mode disabled (cmd=%u%s)",
                     static_cast<unsigned>(cmd),
                     cmd == keyboard_enable_value_ ? ", keyboard takeover" : "");
             }
             return;
         }

         // 4 = 关闭键盘，与 PID 模式状态无关，因此这里不动作。
         if (cmd == keyboard_disable_value_) {
             return;
         }

         RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
             "[MC] unknown mode command: %u", static_cast<unsigned>(cmd));
     }

     bool disable_pid_output() {
         bool was_enabled = false;
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             was_enabled = pid_mode_enabled_;
             pid_mode_enabled_ = false;
             target_ = TargetSetpoint{};
             last_cmd_time_ = std::chrono::steady_clock::now();
         }
         if (was_enabled) {
             reset_all_pid();
             yaw_target_initialized_ = false;
             prev_yaw_target_ = 0.0;
             filtered_yaw_rate_ = 0.0;
         }
         return was_enabled;
     }

     void remote_cb(const hal::msg::HalRemoteControl::SharedPtr msg) {
         if (!active_) return;

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
                 "[MC] /hal/remotecontrol contains a non-finite value");
             return;
         }

         TargetSetpoint sp;
         sp.vx    = normalize_remote_channel(channels[0]) * 1.0;   // m/s
         sp.vy    = normalize_remote_channel(channels[1]) * 0.5;   // m/s
         sp.depth = map_remote_depth(channels[2]);                 // m
         sp.yaw   = normalize_remote_channel(channels[3]) * M_PI;  // rad
         sp.pitch = 0.0;
         sp.roll  = 0.0;
         sp.valid = true;

         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             if (!pid_mode_enabled_) return;
             target_ = sp;
             last_cmd_time_ = std::chrono::steady_clock::now();
         }
     }
 
     // ========================================================================
     // 主控制循环 (定时器回调)
     // ========================================================================
     void control_loop() {
         if (!active_) return;
 
         // 1. 获取最新目标 & 状态 (快照)
         TargetSetpoint target;
         VehicleState   state;
         std::chrono::steady_clock::time_point last_cmd;
         bool pid_mode_enabled = false;
         {
             std::lock_guard<std::mutex> lock1(cmd_mutex_);
             target   = target_;
             last_cmd = last_cmd_time_;
             pid_mode_enabled = pid_mode_enabled_;
         }
         // PID 未开启时完全不发布，避免与键盘遥控节点争抢 /hal/thruster/cmd。
         if (!pid_mode_enabled) {
             return;
         }
         {
             std::lock_guard<std::mutex> lock2(state_mutex_);
             state = state_;
         }
 
         // 2. 看门狗: 超时未收到指令 → 零推力安全停机
         const double dt_cmd = std::chrono::duration<double>(
             std::chrono::steady_clock::now() - last_cmd).count();
         if (!target.valid || dt_cmd > cmd_timeout_s_) {
             if (target.valid) {
                 RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                     "[MC] 指令超时 (%.1fs > %.1fs) → 零推力停机", dt_cmd, cmd_timeout_s_);
             }
             send_zero_thrust();
             return;
         }
 
         // 3. 传感器有效性检查
         if (!state.imu_valid || !state.depth_valid) {
             RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                 "[MC] 传感器数据无效 (imu=%d depth=%d) → 零推力停机",
                 state.imu_valid, state.depth_valid);
             send_zero_thrust();
             return;
         }
 
         // 4. 计算期望合力/力矩
         Wrench tau_ff{};
         Wrench tau_pid{};
 
         if (en_ff_)  tau_ff  = compute_feedforward(target, state);
         if (en_pid_) tau_pid = compute_pid(target, state);
 
         Wrench tau_des;
         tau_des.Fx = tau_ff.Fx + tau_pid.Fx;
         tau_des.Fy = tau_ff.Fy + tau_pid.Fy;
         tau_des.Fz = tau_ff.Fz + tau_pid.Fz;
         tau_des.Mx = tau_ff.Mx + tau_pid.Mx;
         tau_des.My = tau_ff.My + tau_pid.My;
         tau_des.Mz = tau_ff.Mz + tau_pid.Mz;
 
         // 5. 控制分配: wrench → thruster %
         auto thruster_cmds = allocate_thrust(tau_des);
 
         // 6. 发布推进器指令
         auto cmd_msg = std_msgs::msg::Float64MultiArray();
         cmd_msg.data.resize(6);  // [主推0x301, 辅推ID2, ID3, ID4, ID5, ID6]
         cmd_msg.data[0] = thruster_cmds[0];             // 推进器1: 主推 CAN ID 0x301
         for (size_t i = 0; i < 5; ++i) {
             cmd_msg.data[i + 1] = thruster_cmds[i + 1]; // 推进器2~6: 辅推电调 ID2~ID6
         }
         thruster_cmd_pub_->publish(cmd_msg);
 
         // 7. [框架] 舵机指令 (不使能时发布零位)
         publish_servo_commands(target, state);
 
         // 8. 缓存当前推力指令 (用于状态反馈)
         {
             std::lock_guard<std::mutex> lock(state_mutex_);
             state_.main_thrust_pct = thruster_cmds[0];
             for (size_t i = 0; i < 5; ++i) {
                 state_.aux_thrust_pct[i] = thruster_cmds[i + 1];
             }
         }
     }
 
     // ========================================================================
     // 前馈计算 (动力学模型补偿)
     // ========================================================================
     Wrench compute_feedforward(const TargetSetpoint & target, const VehicleState & /*state*/) {
         Wrench ff;
 
         // -- 纵荡方向: 线性 + 二次阻尼补偿 --
         double vx_des = en_surge_ ? target.vx : 0.0;
         ff.Fx = ff_drag_lin_x_ * vx_des
               + ff_drag_quad_x_ * vx_des * std::abs(vx_des);
 
         // -- 横荡方向: 线性 + 二次阻尼补偿 --
         double vy_des = en_sway_ ? target.vy : 0.0;
         ff.Fy = ff_drag_lin_y_ * vy_des
               + ff_drag_quad_y_ * vy_des * std::abs(vy_des);
 
         // -- 垂荡方向: 仅浮力微调配平 (深度由PID闭环控制) --
         ff.Fz = ff_buoyancy_;
 
         // -- 横摇/纵倾: 无前馈 (PID单独抑制) --
         ff.Mx = 0.0;
         ff.My = 0.0;
 
         // -- 转艏方向: 线性 + 二次阻尼补偿 --
         // 目标角速度取自近期姿态差分 (稳定状态下接近0, 前馈只在大机动时起作用)
         double yaw_rate_des = estimate_yaw_rate_from_target(target);
         ff.Mz = ff_drag_lin_yaw_ * yaw_rate_des
               + ff_drag_quad_yaw_ * yaw_rate_des * std::abs(yaw_rate_des);
 
         return ff;
     }
 
     // ========================================================================
     // PID反馈计算 (六通道独立)
     // ========================================================================
     Wrench compute_pid(const TargetSetpoint & target, const VehicleState & state) {
         double dt = control_period_s_;
         Wrench pid;
 
         // --- Surge (vx) ---
         if (en_surge_) {
             double err = target.vx - state.vx;
             pid.Fx = pid_vx_.update(err, dt);
         }
 
         // --- Sway (vy) ---
         if (en_sway_) {
             double err = target.vy - state.vy;
             pid.Fy = pid_vy_.update(err, dt);
         }
 
         // --- Depth (位置式, 仅使用深度反馈) ---
         if (en_depth_) {
             double err = target.depth - state.depth;
             pid.Fz = pid_depth_.update(err, dt);
         }
 
         // --- Yaw (艏向, 角度误差带 wrapping) ---
         if (en_yaw_) {
             double err = wrap_angle(target.yaw - state.yaw);
             pid.Mz = pid_yaw_.update(err, dt);
         }
 
         // --- Pitch (纵倾锁定为0) ---
         if (en_pitch_) {
             double err = wrap_angle(0.0 - state.pitch);
             pid.My = pid_pitch_.update(err, dt);
         }
 
         // --- Roll (横摇锁定为0) ---
         if (en_roll_) {
             double err = wrap_angle(0.0 - state.roll);
             pid.Mx = pid_roll_.update(err, dt);
         }
 
         return pid;
     }
 
     // ========================================================================
     // 控制分配: 加权伪逆 T⁺ 将期望 wrench 映射到推进器指令
     //
     //   τ = T·u   (6×6 分配矩阵 × 6维推力向量 = 6维合力)
     //   u = T⁺·τ  (Moore-Penrose 伪逆)
     //
     // 使用阻尼最小二乘 (Damped Least Squares) 以增强数值稳定性:
     //   T⁺ = T^T · (T · T^T + λ²·I)^{-1}
     // ========================================================================
     std::array<double, 6> allocate_thrust(const Wrench & tau) {
         constexpr int N_DOF  = 6;
         constexpr int N_THR  = 6;  // 1主推 + 5辅推
         const double lambda  = 0.01;  // 阻尼因子 (抑制奇异值附近的异常放大)
 
         // 1. 组装 τ 向量
         double tau_arr[N_DOF] = {tau.Fx, tau.Fy, tau.Fz, tau.Mx, tau.My, tau.Mz};
 
         // 2. 计算 T·T^T + λ²I  (6×6 对称矩阵)
         double M[N_DOF][N_DOF] = {};
         for (int i = 0; i < N_DOF; ++i) {
             for (int j = 0; j < N_DOF; ++j) {
                 double sum = 0.0;
                 for (int k = 0; k < N_THR; ++k) {
                     sum += alloc_vec_[i * N_THR + k] * alloc_vec_[j * N_THR + k];
                 }
                 M[i][j] = sum;
                 if (i == j) M[i][j] += lambda * lambda;
             }
         }
 
         // 3. Cholesky 分解 M = L·L^T (M 对称正定)
         double L[N_DOF][N_DOF] = {};
         for (int i = 0; i < N_DOF; ++i) {
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
 
         // 4. 前代/回代: 解 M·y = τ → y
         double y[N_DOF] = {};
         // 前代: L·z = τ
         double z[N_DOF] = {};
         for (int i = 0; i < N_DOF; ++i) {
             double sum = tau_arr[i];
             for (int j = 0; j < i; ++j) sum -= L[i][j] * z[j];
             z[i] = sum / L[i][i];
         }
         // 回代: L^T·y = z
         for (int i = N_DOF - 1; i >= 0; --i) {
             double sum = z[i];
             for (int j = i + 1; j < N_DOF; ++j) sum -= L[j][i] * y[j];
             y[i] = sum / L[i][i];
         }
 
         // 5. u = T^T · y
         std::array<double, N_THR> u{};
         for (int k = 0; k < N_THR; ++k) {
             double sum = 0.0;
             for (int i = 0; i < N_DOF; ++i) {
                 sum += alloc_vec_[i * N_THR + k] * y[i];
             }
             u[k] = sum;
         }
 
         // 6. 输出限幅
         for (int k = 0; k < N_THR; ++k) {
             u[k] = std::clamp(u[k], thrust_min_pct_, thrust_max_pct_);
         }
 
         return u;
     }
 
     // ========================================================================
     // [框架] 舵机指令发布
     // ========================================================================
     void publish_servo_commands(const TargetSetpoint & /*target*/,
                                 const VehicleState & /*state*/) {
         if (!en_servo_) {
             // 不使能时周期发布零位, 保持舵机回中锁力
             auto tail_zero = std_msgs::msg::Float64MultiArray();
             tail_zero.data = {0.0, 0.0, 0.0, 0.0};
             tail_cmd_pub_->publish(tail_zero);
 
             auto wing_zero = std_msgs::msg::Float64MultiArray();
             wing_zero.data = {0.0, 0.0};
             wing_cmd_pub_->publish(wing_zero);
             return;
         }
 
         // Servo allocation remains disabled until the actuator mapping is defined.
         // 例如: 根据期望俯仰/横摇力矩计算尾舵/翼舵偏角
     }
 
     // ========================================================================
     // 安全工具
     // ========================================================================
     void send_zero_thrust() {
         if (!thruster_cmd_pub_ || !thruster_cmd_pub_->is_activated()) {
             reset_all_pid();
             return;
         }
         auto zero_msg = std_msgs::msg::Float64MultiArray();
         zero_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 1主推 + 5辅推
         thruster_cmd_pub_->publish(zero_msg);
         reset_all_pid();
     }
 
     void clear_target_state() {
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             target_ = TargetSetpoint{};
             last_cmd_time_ = std::chrono::steady_clock::now();
         }
         yaw_target_initialized_ = false;
         prev_yaw_target_ = 0.0;
         filtered_yaw_rate_ = 0.0;
     }
 
     void clear_vehicle_state() {
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_ = VehicleState{};
     }
 
     /** @brief 基于目标艏向序列差分估计期望转艏角速度 */
     double estimate_yaw_rate_from_target(const TargetSetpoint & target) {
         if (!yaw_target_initialized_) {
             yaw_target_initialized_ = true;
             prev_yaw_target_ = target.yaw;
             filtered_yaw_rate_ = 0.0;
             return 0.0;
         }
         const double rate = wrap_angle(target.yaw - prev_yaw_target_) / control_period_s_;
         prev_yaw_target_ = target.yaw;
         // 低通滤波抑制阶跃噪声
         filtered_yaw_rate_ = 0.6 * filtered_yaw_rate_ + 0.4 * rate;
         return filtered_yaw_rate_;
     }
 
     // ========================================================================
     // 成员变量
     // ========================================================================
 
     // -- 状态 --
     std::atomic<bool> active_{false};
     VehicleState      state_;
     std::mutex        state_mutex_;
     TargetSetpoint    target_;
     std::chrono::steady_clock::time_point last_cmd_time_{};
     std::mutex        cmd_mutex_;
     bool yaw_target_initialized_ = false;
     double prev_yaw_target_ = 0.0;
     double filtered_yaw_rate_ = 0.0;
 
     // -- PID 控制器 (6通道) --
     PidController pid_vx_;
     PidController pid_vy_;
     PidController pid_depth_;
     PidController pid_yaw_;
     PidController pid_pitch_;
     PidController pid_roll_;
 
     // -- 前馈参数 --
     double ff_drag_lin_x_    = 0.0;
     double ff_drag_quad_x_   = 0.0;
     double ff_drag_lin_y_    = 0.0;
     double ff_drag_quad_y_   = 0.0;
     double ff_drag_lin_z_    = 0.0;
     double ff_drag_quad_z_   = 0.0;
     double ff_drag_lin_yaw_  = 0.0;
     double ff_drag_quad_yaw_ = 0.0;
     double ff_buoyancy_      = 0.0;
 
     // -- 控制分配 --
     std::vector<double> alloc_vec_;  // 6×6 = 36 元素, 行优先
 
     // -- 限幅 --
     double thrust_max_pct_ = 100.0;
     double thrust_min_pct_ = -100.0;
     double control_period_s_ = 0.02;
 
     // -- 使能开关 --
     bool en_surge_  = true;
     bool en_sway_   = true;
     bool en_depth_  = true;
     bool en_yaw_    = true;
     bool en_pitch_  = true;
     bool en_roll_   = true;
     bool en_ff_     = true;
     bool en_pid_    = true;
     bool en_servo_  = false;
     double cmd_timeout_s_ = 1.0;
     bool pid_mode_enabled_ = false;
     uint8_t pid_enable_value_ = 1;
     uint8_t pid_disable_value_ = 2;
     uint8_t keyboard_enable_value_ = 3;
     uint8_t keyboard_disable_value_ = 4;
 
     // -- ROS2 接口 --
     rclcpp::Subscription<hal::msg::HalInertialnavi>::SharedPtr  imu_sub_;
     rclcpp::Subscription<hal::msg::HalDvl>::SharedPtr           dvl_sub_;
     rclcpp::Subscription<hal::msg::HalDepthsensor>::SharedPtr   depth_sub_;
     rclcpp::Subscription<hal::msg::HalMainthruster>::SharedPtr  main_thruster_sub_;
     rclcpp::Subscription<hal::msg::HalAuxithruster>::SharedPtr  aux_thruster_sub_;
     rclcpp::Subscription<hal::msg::HalTailservo>::SharedPtr     tail_servo_sub_;
     rclcpp::Subscription<hal::msg::HalWingservo>::SharedPtr     wing_servo_sub_;
     rclcpp::Subscription<hal::msg::HalModeControl>::SharedPtr       mode_sub_;
     rclcpp::Subscription<hal::msg::HalRemoteControl>::SharedPtr     remote_sub_;
 
     std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
         thruster_cmd_pub_;
     std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
         tail_cmd_pub_;
     std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>>
         wing_cmd_pub_;
 
     rclcpp::TimerBase::SharedPtr control_timer_;
 };
 
 // ============================================================================
 // main
 // ============================================================================
 int main(int argc, char ** argv) {
     rclcpp::init(argc, argv);
     auto node = std::make_shared<BspMotionControlNode>("bsp_motioncontrol_node");
     rclcpp::executors::SingleThreadedExecutor executor;
     executor.add_node(node->get_node_base_interface());
     executor.spin();
     rclcpp::shutdown();
     return 0;
 }
 
