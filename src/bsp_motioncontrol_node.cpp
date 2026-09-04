/**
 * @file bsp_motioncontrol_node.cpp
 * @brief BSP层运动控制节点 —— 6-DOF FF+PID+DOB 闭环控制 (控制律/分配自 bsp_py 移植)
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
 * ## 控制算法 (逐自由度, 自 bsp_py dof_controller.py 移植)
 *   τ_i = τ_FF,i + τ_PID,i − d̂_i,  按 controller_mode 分级:
 *     1=纯PID, 2=FF+PID (默认), 3=FF+PID+DOB
 *   前馈: surge/sway 用 ν_des 线性+二次阻尼前馈; depth 用浮力配平
 *         buoyancy_trim; 角通道前馈为 0
 *   PID: 六通道离散PID, 积分抗饱和 + D项一阶低通 (pid_deriv_alpha)
 *   DOB: d̂ = α·d̂₋ + (1−α)·[τ_prev − m_eff·ν̇ − d_eff·ν], α=exp(−带宽·dt)
 *         ν̇ 后向差分; 角通道 ν = p/q/r (姿态差分 + J2_inv 估计)
 *   分配: τ (N/N·m) → u (N), 逐次截断阻尼最小二乘, |u_k|≤thrust_limits[k],
 *         死区后按各推上限换算百分比发布
 *   门控 (D5): IMU/DVL/深度计任一无效或推进器故障 → 零推力全停;
 *         通道 enable=false 时该通道 PID/FF/DOB 全部冻结
 *   参数现状: thrust_limits/mass_eff/damp_eff/alloc_matrix 为 python 移植占位值,
 *         以 ⚠ 标注, 待实测/模型替换后再做水池整定
 * ## 在线调参 (v0.2, 无需重新编译/重启)
 *   全部控制参数在运行期可通过 `ros2 param set /bsp_motioncontrol_node <参数> <值>`
 *   实时修改并即时生效: PID 增益/限幅、前馈系数、分配矩阵 alloc_matrix、分配阻尼
 *   alloc_lambda、微分滤波系数 pid_deriv_alpha、各通道使能开关、推力/舵机限幅、
 *   看门狗超时 cmd_timeout_s、控制频率 control_rate_hz、模式命令值等。
 *   非法值/非法组合会被整体拒绝并返回原因, 不会产生半生效状态;
 *   话题类参数 (mode_topic/remote_topic) 在下次 configure 时生效。
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
 #include "rcl_interfaces/msg/set_parameters_result.hpp"
 #include "rclcpp/node_interfaces/node_parameters_interface.hpp"
 #include "rclcpp/parameter.hpp"
 #include <initializer_list>
 
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
 // 扰动观测器 DOB (移植自 bsp_py dof_controller.py)
 //   d̂ = α·d̂₋ + (1−α)·[τ_prev − m_eff·ν̇ − d_eff·ν],   α = exp(−L·dt)
 // 补偿未建模扰动: 海流/浮力差/缆力/模型误差。L=0 时禁用。
 // ============================================================================
 struct DobController {
     double bandwidth = 0.0;   // 观测器带宽 L (rad/s), 0 = 禁用
     double mass_eff  = 1.0;   // 等效质量/惯量 (kg 或 kg·m²)
     double damp_eff  = 0.0;   // 等效线性阻尼

     double d_hat       = 0.0; // 扰动估计
     double alpha       = 0.0; // 低通系数 α
     double prev_nu     = 0.0; // 上一拍速度 (ν̇ 后向差分)
     bool   has_prev_nu = false;
     bool   alpha_valid = false;

     void configure(double bw, double mass, double damp)
     {
         bandwidth   = bw;
         mass_eff    = mass;
         damp_eff    = damp;
         alpha_valid = false;
     }

     void reset()
     {
         d_hat       = 0.0;
         alpha       = 0.0;
         prev_nu     = 0.0;
         has_prev_nu = false;
         alpha_valid = false;
     }

     /** 控制频率变化: α 与 ν̇ 差分基准失效 */
     void on_dt_change()
     {
         alpha_valid = false;
         has_prev_nu = false;
     }

     /** 一步更新, 返回当前扰动估计 (禁用/无效 dt 返回 0) */
     double update(double nu, double tau_prev, double dt)
     {
         if (bandwidth <= 0.0 || dt <= 0.0) return 0.0;
         if (!alpha_valid) {
             alpha = std::exp(-bandwidth * dt);
             alpha_valid = true;
         }
         const double nu_dot = has_prev_nu ? (nu - prev_nu) / dt : 0.0;
         prev_nu     = nu;
         has_prev_nu = true;
         const double raw_d = tau_prev - mass_eff * nu_dot - damp_eff * nu;
         d_hat = alpha * d_hat + (1.0 - alpha) * raw_d;
         return d_hat;
     }
 };

 // python 自由度顺序 (surge,sway,depth,yaw,pitch,roll) → wrench 行 (Fx,Fy,Fz,Mx,My,Mz)
 static constexpr int DOF_ROW[6] = {0, 1, 2, 5, 4, 3};

 
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

         // ----- 在线调参: 全局算法系数 (原硬编码, 现开放为运行期参数) -----
         this->declare_parameter<double>("pid_deriv_alpha", 0.7);
         this->declare_parameter<double>("alloc_lambda", 0.01);
         this->declare_parameter<int>("controller_mode", 2);   // 1=纯PID 2=FF+PID 3=FF+PID+DOB
         this->declare_parameter<double>("deadzone_pct", 3.0); // 分配死区 (% of u_max)
         this->declare_parameter<bool>("imu_deg2rad", true);        // 惯导欧拉角(度)转弧度
         this->declare_parameter<double>("imu_yaw_offset_deg", 0.0); // 航向安装偏置(度)
         this->declare_parameter<double>("servo_cmd_hz", 5.0);   // 舵机指令发送频率(Hz); 0=仅数值变化时发
         this->declare_parameter<std::vector<double>>("thrust_limits",
             {441.0, 69.0, 69.0, 69.0, 69.0, 69.0});  // 各推推力上限 (N), ⚠ python 占位
         // DOB 系数 (python 命名; ⚠ 数值占位待实测, roll 量级疑似异常)
         this->declare_parameter<double>("surge.dob_bandwidth", 3.0);
         this->declare_parameter<double>("surge.mass_eff", 275.0);
         this->declare_parameter<double>("surge.damp_eff", 2.5);   // Dl 实测 (线性阻尼)
         this->declare_parameter<double>("sway.dob_bandwidth", 3.0);
         this->declare_parameter<double>("sway.mass_eff", 526.8);
         this->declare_parameter<double>("sway.damp_eff", 125.0);
         this->declare_parameter<double>("depth.dob_bandwidth", 2.0);
         this->declare_parameter<double>("depth.mass_eff", 526.8);
         this->declare_parameter<double>("depth.damp_eff", 125.0);
         this->declare_parameter<double>("yaw.dob_bandwidth", 2.0);
         this->declare_parameter<double>("yaw.mass_eff", 472.9);
         this->declare_parameter<double>("yaw.damp_eff", 47.2);
         this->declare_parameter<double>("pitch.dob_bandwidth", 1.5);
         this->declare_parameter<double>("pitch.mass_eff", 472.0);
         this->declare_parameter<double>("pitch.damp_eff", 47.2);
         this->declare_parameter<double>("roll.dob_bandwidth", 1.5);
         this->declare_parameter<double>("roll.mass_eff", 4.345);   // 细长回转体, 模型提供 (横摇惯量小合理)
         this->declare_parameter<double>("roll.damp_eff", 0.434);   // Dl 实测
 
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
         this->declare_parameter<double>("ff.drag_linear_x", 2.5);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_quadratic_x", 10.0);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_linear_y", 125.0);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_quadratic_y", 747.0);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_linear_z", 125.0);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_quadratic_z", 747.0);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_linear_yaw", 47.2);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.drag_quadratic_yaw", 1262.0);  // Dl/Dnl 实测
         this->declare_parameter<double>("ff.buoyancy_trim",   0.0);
 
         // ----- 控制分配矩阵 T (6×6, 行优先) -----
         //  T ∈ R^(6×6):  τ = T · u,  u 单位 N (各推), 行序 [Fx,Fy,Fz,Mx,My,Mz]
         // 列/输出顺序: [主推0x301, ID2, ID3, ID4, ID5, ID6]
         // 真实布局 (2026-09 总体提供, 无斜装):
         //   ID1主推 x≈0; ID2侧推 x=-1.78283(后); ID3/4垂推 x=-1.6631(后), y=∓0.075(间距150mm);
         //   ID5垂推 x=+1.6236(前); ID6侧推 x=+1.74559(前)
         // 约定: x 前为正, z 向下为正; 正指令=该推沿体轴产生正向力 (主推+x, 侧推+y, 垂推+z)
         // 力矩按 τ=r×F: Mx=y·Fz, My=-x·Fz, Mz=x·Fy
         // ⚠ 指令符号与物理推力方向的最终对应以台架正反转验证为准;
         //   若某推实测反向, 翻转 alloc_matrix 中该列全部符号即可 (改参数, 无需重编)
         std::vector<double> default_T = {
             //  Main     ID2       ID3      ID4      ID5      ID6
                 1.0,    0.0,      0.0,     0.0,     0.0,     0.0,      // Fx  主推
                 0.0,    1.0,      0.0,     0.0,     0.0,     1.0,      // Fy  ID2+ID6 同向=横荡
                 0.0,    0.0,      1.0,     1.0,     1.0,     0.0,      // Fz  ID3+ID4+ID5 垂推
                 0.0,    0.0,     -0.075,   0.075,   0.0,     0.0,      // Mx  y·Fz (ID3=-y, ID4=+y)
                 0.0,    0.0,      1.6631,  1.6631, -1.6236,  0.0,      // My  -x·Fz (后推/前推差动=纵倾)
                 0.0,   -1.78283,  0.0,     0.0,     0.0,     1.74559,  // Mz  x·Fy (后/前侧推差动=转艏)
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

         // ----- 在线调参: 注册参数变更回调 (ros2 param set 即时生效) -----
         param_cb_handle_ = this->add_on_set_parameters_callback(
             [this](const std::vector<rclcpp::Parameter> & params) {
                 return on_parameter_set(params);
             });
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
             reset_all_controllers();
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
             reset_all_controllers();
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
             reset_all_controllers();
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
         // 微分滤波系数 & 分配阻尼 (运行期可通过参数在线修改, 此处加载启动值)
         pid_deriv_alpha_ = this->get_parameter("pid_deriv_alpha").as_double();
         alloc_lambda_    = this->get_parameter("alloc_lambda").as_double();
         if (!std::isfinite(pid_deriv_alpha_) || pid_deriv_alpha_ < 0.0 ||
             pid_deriv_alpha_ >= 1.0) {
             throw std::invalid_argument(
                 "pid_deriv_alpha must be finite and in [0, 1)");
         }
         if (!std::isfinite(alloc_lambda_) || alloc_lambda_ <= 0.0 ||
             alloc_lambda_ > 1000.0) {
             throw std::invalid_argument(
                 "alloc_lambda must be finite and in (0, 1000]");
         }

         // 控制律模式 / 死区 / 推力上限 / DOB 系数 (python 版移植)
         controller_mode_ = static_cast<int>(this->get_parameter("controller_mode").as_int());
         deadzone_pct_ = this->get_parameter("deadzone_pct").as_double();
         if (controller_mode_ < 1 || controller_mode_ > 3 ||
             !std::isfinite(deadzone_pct_) || deadzone_pct_ < 0.0 ||
             deadzone_pct_ > 100.0) {
             throw std::invalid_argument(
                 "controller_mode must be 1/2/3 and deadzone_pct in [0,100]");
         }
         imu_deg2rad_    = this->get_parameter("imu_deg2rad").as_bool();
         imu_yaw_offset_ = this->get_parameter("imu_yaw_offset_deg").as_double() * M_PI / 180.0;
         if (!std::isfinite(imu_yaw_offset_)) {
             throw std::invalid_argument("imu_yaw_offset_deg must be finite");
         }
         servo_cmd_hz_ = this->get_parameter("servo_cmd_hz").as_double();
         if (!std::isfinite(servo_cmd_hz_) || servo_cmd_hz_ < 0.0 || servo_cmd_hz_ > 200.0) {
             throw std::invalid_argument("servo_cmd_hz must be in [0, 200]");
         }
         const std::vector<double> tlim = this->get_parameter("thrust_limits").as_double_array();
         if (tlim.size() != 6U || !std::all_of(tlim.begin(), tlim.end(),
                 [](double x) { return std::isfinite(x) && x > 0.0 && x <= 1e6; })) {
             throw std::invalid_argument("thrust_limits must be 6 values in (0,1e6]");
         }
         for (size_t i = 0; i < 6U; ++i) thrust_limits_[i] = tlim[i];
         static const std::array<std::string, 6> DOF_PARAM = {
             "surge", "sway", "depth", "yaw", "pitch", "roll"};
         for (size_t i = 0; i < 6U; ++i) {
             const std::string pfx = DOF_PARAM[i] + ".";
             const double bw = this->get_parameter(pfx + "dob_bandwidth").as_double();
             const double me = this->get_parameter(pfx + "mass_eff").as_double();
             const double de = this->get_parameter(pfx + "damp_eff").as_double();
             if (!std::isfinite(bw) || bw < 0.0 || bw > 1000.0 ||
                 !std::isfinite(me) || me <= 0.0 ||
                 !std::isfinite(de) || de < 0.0) {
                 throw std::invalid_argument(
                     DOF_PARAM[i] + " DOB coefficients invalid");
             }
             dob_[DOF_ROW[i]].configure(bw, me, de);
         }

         auto load_pid = [this](const std::string & prefix, PidController & pid) {
             pid.kp      = this->get_parameter(prefix + ".kp").as_double();
             pid.ki      = this->get_parameter(prefix + ".ki").as_double();
             pid.kd      = this->get_parameter(prefix + ".kd").as_double();
             pid.max_i   = this->get_parameter(prefix + ".max_i").as_double();
             pid.max_out = this->get_parameter(prefix + ".max_out").as_double();
             pid.alpha   = pid_deriv_alpha_;  // 微分滤波系数 (在线可调)
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
 
     void reset_all_controllers() {
         pid_vx_.reset();   pid_vy_.reset();
         pid_depth_.reset(); pid_yaw_.reset();
         pid_pitch_.reset(); pid_roll_.reset();
        for (DobController & d : dob_) d.reset();
        tau_prev_real_.fill(0.0);
        reset_attitude_estimator();
     
         servo_cmd_sent_once_ = false;   // 激活/复位后下个周期重新发送一次舵机指令
    }
 
     // ========================================================================
     // 传感器回调 (线程安全, 使用 std::atomic / mutex)
     // ========================================================================
     void imu_cb(const hal::msg::HalInertialnavi::SharedPtr msg) {
         if (!active_) return;
         // GI510 UZHDR: Heading/Pitch/Roll 单位为度; 驱动原样透传, 此处转弧度并加安装偏置。
         double yaw   = static_cast<double>(msg->yaw);
         double pitch = static_cast<double>(msg->pitch);
         double roll  = static_cast<double>(msg->roll);
         if (imu_deg2rad_) {
             constexpr double DEG2RAD = M_PI / 180.0;
             yaw   *= DEG2RAD;
             pitch *= DEG2RAD;
             roll  *= DEG2RAD;
         }
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_.yaw   = wrap_angle(yaw + imu_yaw_offset_);
         state_.pitch = pitch;
         state_.roll  = roll;
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
                 reset_all_controllers();
                 reset_attitude_estimator();
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
             reset_all_controllers();
             reset_attitude_estimator();
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
     // ============================================================================
     // 主控制循环 (定时器回调)
     // ============================================================================
     void control_loop()
     {
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
         // PID 未开启时完全不发布, 避免与键盘遥控节点争抢 /hal/thruster/cmd。
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

         // 3. 传感器 & 执行器健康检查
         //    D5 决策: DVL 失效直接全停; 推进器故障直接全停 (与 bsp_py 一致)。
         const bool any_aux_fault = std::any_of(state.aux_fault.begin(),
                                                state.aux_fault.end(),
                                                [](bool f) { return f; });
         if (!state.imu_valid || !state.dvl_valid || !state.depth_valid) {
             RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                 "[MC] 传感器数据无效 (imu=%d dvl=%d depth=%d) → 零推力停机",
                 state.imu_valid, state.dvl_valid, state.depth_valid);
             send_zero_thrust();
             return;
         }
         if (state.main_fault || any_aux_fault) {
             RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                 "[MC] 推进器故障 (main=%d aux=%d) → 零推力停机",
                 state.main_fault, static_cast<int>(any_aux_fault));
             send_zero_thrust();
             return;
         }

         // 4. 估计体坐标角速率 p/q/r (供角通道 DOB, 移植自 bsp_py)
         const std::array<double, 3> pqr =
             estimate_body_rates(state.yaw, state.pitch, state.roll);

         // 5. 期望合力/力矩 (逐自由度 FF+PID+DOB)
         const Wrench tau_des =
             compute_control_wrench(target, state, pqr[0], pqr[1], pqr[2]);

         // 6. 推力分配 (N 单位, 逐次截断 DLS)
         const std::array<double, 6> u_cmd = allocate_thrust(tau_des);

         // 7. 死区 → 实际施加 wrench (喂 DOB) → N 换算百分比发布
         std::array<double, 6> u_eff{};
         std::array<double, 6> thruster_pct{};
         for (size_t k = 0; k < 6U; ++k) {
             u_eff[k] = (std::abs(u_cmd[k]) < deadzone_pct_ * 0.01 * thrust_limits_[k])
                            ? 0.0 : u_cmd[k];
             thruster_pct[k] = std::clamp(u_eff[k] / thrust_limits_[k] * 100.0,
                                          thrust_min_pct_, thrust_max_pct_);
         }
         tau_prev_real_ = apply_allocation(u_eff);

         // 8. 发布推进器指令 (data[0]=主推 0x301, data[1..5]=辅推 ID2..ID6)
         auto cmd_msg = std_msgs::msg::Float64MultiArray();
         cmd_msg.data.assign(thruster_pct.begin(), thruster_pct.end());
         thruster_cmd_pub_->publish(cmd_msg);

         // 9. [框架] 舵机指令 (不使能时发布零位)
         publish_servo_commands(target, state);

         // 10. 缓存当前推力指令百分比 (供状态反馈)
         {
             std::lock_guard<std::mutex> lock(state_mutex_);
             state_.main_thrust_pct = thruster_pct[0];
             for (size_t i = 0; i < 5; ++i) {
                 state_.aux_thrust_pct[i] = thruster_pct[i + 1];
             }
         }
     }

     // ============================================================================
     // 体坐标角速率估计 (欧拉角差分 + J2_inv, 移植自 bsp_py _estimate_body_angular_rates)
     // ============================================================================
     std::array<double, 3> estimate_body_rates(double yaw, double pitch, double roll)
     {
         const auto now = std::chrono::steady_clock::now();
         if (!att_initialized_) {
             att_initialized_ = true;
             prev_yaw_ = yaw;
             prev_pitch_ = pitch;
             prev_roll_ = roll;
             prev_att_time_ = now;
             return {0.0, 0.0, 0.0};
         }
         const double dt = std::chrono::duration<double>(now - prev_att_time_).count();
         if (dt <= 0.0) {
             return {0.0, 0.0, 0.0};
         }
         const double dphi   = wrap_angle(roll - prev_roll_) / dt;
         const double dtheta = wrap_angle(pitch - prev_pitch_) / dt;
         const double dpsi   = wrap_angle(yaw - prev_yaw_) / dt;
         const double st = std::sin(pitch), ct = std::cos(pitch);
         const double cp = std::cos(roll),   sp = std::sin(roll);
         const double p = dphi - st * dpsi;
         const double q = cp * dtheta + sp * ct * dpsi;
         const double r = -sp * dtheta + cp * ct * dpsi;
         prev_yaw_ = yaw;
         prev_pitch_ = pitch;
         prev_roll_ = roll;
         prev_att_time_ = now;
         return {p, q, r};
     }

     /** @brief 姿态估计器复位 (随控制器复位调用) */
     void reset_attitude_estimator()
     {
         att_initialized_ = false;
         prev_yaw_ = 0.0;
         prev_pitch_ = 0.0;
         prev_roll_ = 0.0;
     }

     // ============================================================================
     // 逐自由度 FF+PID+DOB 控制律 (移植自 bsp_py dof_controller.py)
     //   τ = τ_FF + τ_PID − d̂   行序: [Fx Fy Fz Mx My Mz]
     // ============================================================================
     Wrench compute_control_wrench(const TargetSetpoint & target,
                                   const VehicleState & state,
                                   double p, double q, double r)
     {
         Wrench tau{};
         const int    mode = controller_mode_;
         const double dt   = control_period_s_;

         auto pid_ptr = [this](int row) -> PidController* {
             switch (row) {
                 case 0: return &pid_vx_;
                 case 1: return &pid_vy_;
                 case 2: return &pid_depth_;
                 case 3: return &pid_roll_;
                 case 4: return &pid_pitch_;
                 default: return &pid_yaw_;
             }
         };

         // 单自由度一步: 冻结通道整体不更新 (PID/FF/DOB 状态全保留)
         auto run_law = [&](int row, bool enabled, double error,
                            double nu_act, double tau_prev, double tau_ff) -> double {
             if (!enabled) return 0.0;
             double out = 0.0;
             if (en_pid_) {
                 out += pid_ptr(row)->update(error, dt);
             }
             if (en_ff_ && mode >= 2) {
                 out += tau_ff;
             }
             if (mode >= 3) {
                 out -= dob_[row].update(nu_act, tau_prev, dt);
             }
             return out;
         };

         // surge (Fx): 速度环, ν_des 阻尼前馈
         tau.Fx = run_law(0, en_surge_, target.vx - state.vx,
                          state.vx, tau_prev_real_[0],
                          ff_drag_lin_x_ * target.vx +
                              ff_drag_quad_x_ * target.vx * std::abs(target.vx));
         // sway (Fy): 速度环
         tau.Fy = run_law(1, en_sway_, target.vy - state.vy,
                          state.vy, tau_prev_real_[1],
                          ff_drag_lin_y_ * target.vy +
                              ff_drag_quad_y_ * target.vy * std::abs(target.vy));
         // depth (Fz): 位置环, 前馈 = 浮力配平, DOB 观测 ν=vz
         tau.Fz = run_law(2, en_depth_, target.depth - state.depth,
                          state.vz, tau_prev_real_[2], ff_buoyancy_);
         // roll / pitch / yaw: 角度锁定/跟踪, 前馈 0, DOB 观测 p / q / r
         tau.Mx = run_law(3, en_roll_, wrap_angle(0.0 - state.roll),
                          p, tau_prev_real_[3], 0.0);
         tau.My = run_law(4, en_pitch_, wrap_angle(0.0 - state.pitch),
                          q, tau_prev_real_[4], 0.0);
         tau.Mz = run_law(5, en_yaw_, wrap_angle(target.yaw - state.yaw),
                          r, tau_prev_real_[5], 0.0);
         return tau;
     }

     /** @brief 分配后实际施加广义力 T·u (N), 供 DOB 下一拍反馈 */
     std::array<double, 6> apply_allocation(const std::array<double, 6> & u)
     {
         std::array<double, 6> tau{};
         for (int i = 0; i < 6; ++i) {
             double sum = 0.0;
             for (int k = 0; k < 6; ++k) {
                 sum += alloc_vec_[i * 6 + k] * u[k];
             }
             tau[i] = sum;
         }
         return tau;
     }

     // ============================================================================
     // 控制分配: 逐次截断阻尼最小二乘 (移植自 bsp_py thrust_alloc.py)
     //   约束 |u_k| ≤ thrust_limits_[k] (N); 饱和推钉死后对残余 wrench 重分配
     // ============================================================================
     std::array<double, 6> allocate_thrust(const Wrench & tau)
     {
         const std::array<double, 6> tau_arr = {
             tau.Fx, tau.Fy, tau.Fz, tau.Mx, tau.My, tau.Mz};
         std::array<double, 6> u{};
         std::array<bool, 6>   pinned{};
         const double lambda = (std::isfinite(alloc_lambda_) && alloc_lambda_ > 1e-9)
                                   ? alloc_lambda_ : 0.01;

         for (int iter = 0; iter < 6; ++iter) {
             // 未饱和推进器 (free 列)
             int n_free = 0;
             int free_idx[6];
             for (int k = 0; k < 6; ++k) {
                 if (!pinned[k]) free_idx[n_free++] = k;
             }
             if (n_free == 0) break;

             // 残余目标: 减去已钉死列贡献
             std::array<double, 6> tau_rem = tau_arr;
             for (int k = 0; k < 6; ++k) {
                 if (!pinned[k]) continue;
                 for (int i = 0; i < 6; ++i) {
                     tau_rem[i] -= alloc_vec_[i * 6 + k] * u[k];
                 }
             }

             // M = T_f·T_fᵀ + λ²I (6×6), u_f = T_fᵀ·M⁻¹·τ_rem
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
                     else        L[i][j] = sum / L[j][j];
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
                     u[j] = lim; pinned[j] = true; any_sat = true;
                 } else if (val < -lim) {
                     u[j] = -lim; pinned[j] = true; any_sat = true;
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


     // ========================================================================
     // [框架] 舵机指令发布
     // ========================================================================
     void publish_servo_commands(const TargetSetpoint & /*target*/,
                                   const VehicleState & /*state*/) {
         // 舵机指令限频: 避免 50Hz 连续下发造成 CAN 总线与舵机处理拥塞
         // (曾导致 hal_servo_node 看门狗误判断连)。策略:
         //   - 数值变化 → 立即发送;
         //   - 未变化 → 按 servo_cmd_hz_ 周期保持发送 (默认5Hz; 0=不再周期发送)。
         const auto now = std::chrono::steady_clock::now();
         const std::array<double, 4> tail_des = {0.0, 0.0, 0.0, 0.0}; // 未来: 分配输出
         const std::array<double, 2> wing_des = {0.0, 0.0};
         bool changed = !servo_cmd_sent_once_ ||
                        tail_des != last_tail_cmd_sent_ ||
                        wing_des != last_wing_cmd_sent_;
         bool due = false;
         if (servo_cmd_hz_ > 0.0) {
             const double elapsed = std::chrono::duration<double>(
                 now - last_servo_cmd_time_).count();
             due = elapsed >= (1.0 / servo_cmd_hz_);
         }
         if (!changed && !due) return;
     
         auto tail_msg = std_msgs::msg::Float64MultiArray();
         tail_msg.data.assign(tail_des.begin(), tail_des.end());
         tail_cmd_pub_->publish(tail_msg);
         auto wing_msg = std_msgs::msg::Float64MultiArray();
         wing_msg.data.assign(wing_des.begin(), wing_des.end());
         wing_cmd_pub_->publish(wing_msg);
         last_tail_cmd_sent_  = tail_des;
         last_wing_cmd_sent_  = wing_des;
         last_servo_cmd_time_ = now;
         servo_cmd_sent_once_ = true;
     }
 
     // ========================================================================
     // 安全工具
     // ========================================================================
     void send_zero_thrust() {
         if (!thruster_cmd_pub_ || !thruster_cmd_pub_->is_activated()) {
             reset_all_controllers();
             return;
         }
         auto zero_msg = std_msgs::msg::Float64MultiArray();
         zero_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 1主推 + 5辅推
         thruster_cmd_pub_->publish(zero_msg);
         reset_all_controllers();
     }
 
     void clear_target_state() {
         {
             std::lock_guard<std::mutex> lock(cmd_mutex_);
             target_ = TargetSetpoint{};
             last_cmd_time_ = std::chrono::steady_clock::now();
         }
         reset_attitude_estimator();
     }
 
     void clear_vehicle_state() {
         std::lock_guard<std::mutex> lock(state_mutex_);
         state_ = VehicleState{};
     }
 
 
     // ========================================================================
     // 在线调参: 参数变更回调
     // 运行期 `ros2 param set` / `ros2 param load` 即时生效, 无需重新编译/重启。
     // 两阶段提交: 阶段1 校验全部变更 (任一非法则整批拒绝), 阶段2 统一应用。
     // 说明: 本节点在单线程 executor 中运行, 参数回调与订阅/定时器回调互斥,
     //       因此直接修改成员变量无数据竞争。
     // ========================================================================

     /** @brief 拆分 PID 参数名, 形如 pid_vx.kp → dof="vx", key="kp". */
     static bool split_pid_param_name(const std::string & name,
                                      std::string & dof, std::string & key)
     {
         static const std::array<std::string, 6> DOFS = {
             "vx", "vy", "depth", "yaw", "pitch", "roll"};
         static const std::array<std::string, 5> KEYS = {
             "kp", "ki", "kd", "max_i", "max_out"};
         for (const std::string & d : DOFS) {
             const std::string prefix = "pid_" + d + ".";
             if (name.rfind(prefix, 0) != 0) continue;
             const std::string k = name.substr(prefix.size());
             if (std::find(KEYS.begin(), KEYS.end(), k) == KEYS.end()) return false;
             dof = d;
             key = k;
             return true;
         }
         return false;
     }

     /** @brief 按自由度名取对应 PID 控制器. */
     PidController * pid_of_dof(const std::string & dof)
     {
         if (dof == "vx")    return &pid_vx_;
         if (dof == "vy")    return &pid_vy_;
         if (dof == "depth") return &pid_depth_;
         if (dof == "yaw")   return &pid_yaw_;
         if (dof == "pitch") return &pid_pitch_;
         return &pid_roll_;
     }

     /** @brief 是否为前馈参数名. */
     static bool is_ff_param_name(const std::string & name)
     {
         static const std::array<std::string, 9> FF_NAMES = {
             "ff.drag_linear_x",     "ff.drag_quadratic_x",
             "ff.drag_linear_y",     "ff.drag_quadratic_y",
             "ff.drag_linear_z",     "ff.drag_quadratic_z",
             "ff.drag_linear_yaw",   "ff.drag_quadratic_yaw",
             "ff.buoyancy_trim"};
         return std::find(FF_NAMES.begin(), FF_NAMES.end(), name) != FF_NAMES.end();
     }

     /** @brief 拆分 DOB 参数名 (python 命名如 surge.dob_bandwidth) → wrench 行 & 键 */
     static bool split_dob_param_name(const std::string & name, int & row, int & key)
     {
         static const std::array<std::string, 6> DOFS = {
             "surge", "sway", "depth", "yaw", "pitch", "roll"};
         static const std::array<std::string, 3> KEYS = {
             "dob_bandwidth", "mass_eff", "damp_eff"};
         for (size_t i = 0; i < DOFS.size(); ++i) {
             const std::string prefix = DOFS[i] + ".";
             if (name.rfind(prefix, 0) != 0) continue;
             for (size_t k = 0; k < KEYS.size(); ++k) {
                 if (name == prefix + KEYS[k]) {
                     row = DOF_ROW[i];
                     key = static_cast<int>(k);
                     return true;
                 }
             }
         }
         return false;
     }


     /** @brief 数值参数读取: 兼容 double 与 integer 两种类型. */
     static double param_to_double(const rclcpp::Parameter & p)
     {
         try { return p.as_double(); } catch (const std::exception &) { }
         try { return static_cast<double>(p.as_int()); } catch (const std::exception &) { }
         throw std::invalid_argument("参数必须是数值类型 (double/integer)");
     }

     /** @brief 整数参数读取: integer 优先, 兼容整数值的 double. */
     static int64_t param_to_int(const rclcpp::Parameter & p)
     {
         try { return p.as_int(); } catch (const std::exception &) { }
         try {
             const double v = p.as_double();
             if (std::isfinite(v) && v == std::floor(v)) {
                 return static_cast<int64_t>(v);
             }
         } catch (const std::exception &) { }
         throw std::invalid_argument("参数必须是整数类型");
     }

     /** @brief 应用单个前馈参数. */
     void set_ff_param(const std::string & name, double v)
     {
         if      (name == "ff.drag_linear_x")     ff_drag_lin_x_     = v;
         else if (name == "ff.drag_quadratic_x")  ff_drag_quad_x_    = v;
         else if (name == "ff.drag_linear_y")     ff_drag_lin_y_     = v;
         else if (name == "ff.drag_quadratic_y")  ff_drag_quad_y_    = v;
         else if (name == "ff.drag_linear_z")     ff_drag_lin_z_     = v;
         else if (name == "ff.drag_quadratic_z")  ff_drag_quad_z_    = v;
         else if (name == "ff.drag_linear_yaw")   ff_drag_lin_yaw_   = v;
         else if (name == "ff.drag_quadratic_yaw") ff_drag_quad_yaw_ = v;
         else if (name == "ff.buoyancy_trim")     ff_buoyancy_       = v;
     }

     /** @brief 应用单个 PID 增益/限幅参数. */
     void set_pid_param_value(const std::string & dof, const std::string & key, double v)
     {
         PidController * c = pid_of_dof(dof);
         if      (key == "kp")      c->kp      = v;
         else if (key == "ki")      c->ki      = v;
         else if (key == "kd")      c->kd      = v;
         else if (key == "max_i")   c->max_i   = v;
         else if (key == "max_out") c->max_out = v;
     }

     /** @brief 控制频率变化后重建控制定时器. */
     void restart_control_timer()
     {
         // 控制频率变化: DOB 的 α 与 ν̇ 差分基准需重建
         for (DobController & d : dob_) d.on_dt_change();
         if (!control_timer_) return;  // configure 前无需处理, configure 时按最新频率创建
         control_timer_->cancel();
         control_timer_.reset();
         const double rate = 1.0 / control_period_s_;
         const int period_ms = std::max(1, static_cast<int>(1000.0 / rate));
         control_timer_ = this->create_wall_timer(
             std::chrono::milliseconds(period_ms),
             std::bind(&BspMotionControlNode::control_loop, this));
         RCLCPP_INFO(get_logger(), "[MC] 控制频率已在线更新: %.1f Hz", rate);
     }

     /** @brief ROS2 参数回调: 运行期在线调参入口. */
     rcl_interfaces::msg::SetParametersResult
     on_parameter_set(const std::vector<rclcpp::Parameter> & params)
     {
         const auto reject = [](const std::string & reason) {
             rcl_interfaces::msg::SetParametersResult r;
             r.successful = false;
             r.reason     = reason;
             return r;
         };

         // ---- 阶段 1: 校验全部变更 (任一非法 → 整批拒绝, 不产生半生效状态) ----
         double new_alpha       = pid_deriv_alpha_;
         double new_lambda      = alloc_lambda_;
         double new_rate        = 1.0 / control_period_s_;
         double new_cmd_timeout = cmd_timeout_s_;
         double new_thrust_min  = thrust_min_pct_;
         double new_thrust_max  = thrust_max_pct_;
         bool   thrust_touched  = false;
         bool   modes_touched   = false;
         std::array<int64_t, 4> new_modes = {
             pid_enable_value_, pid_disable_value_,
             keyboard_enable_value_, keyboard_disable_value_};
         std::vector<rclcpp::Parameter> accepted;
         accepted.reserve(params.size());

         for (const rclcpp::Parameter & p : params) {
             const std::string & n = p.get_name();
             try {
                 int dob_row = -1;
                 int dob_key = -1;
                 const bool is_dob = split_dob_param_name(n, dob_row, dob_key);
                 if (n == "pid_deriv_alpha" || n == "alloc_lambda") {
                     const double v = param_to_double(p);
                     if (n == "pid_deriv_alpha") {
                         if (!std::isfinite(v) || v < 0.0 || v >= 1.0) {
                             return reject(n + ": 需满足 [0, 1)");
                         }
                         new_alpha = v;
                     } else {
                         if (!std::isfinite(v) || v <= 0.0 || v > 1000.0) {
                             return reject(n + ": 需满足 (0, 1000]");
                         }
                         new_lambda = v;
                     }
                 } else if (n == "alloc_matrix") {
                     const std::vector<double> v = p.as_double_array();
                     if (v.size() != 36U) {
                         return reject(n + ": 必须恰好 36 个元素");
                     }
                     if (!std::all_of(v.begin(), v.end(),
                             [](double x) { return std::isfinite(x); })) {
                         return reject(n + ": 元素必须全部为有限值");
                     }
                 } else if (n == "thrust_min_pct" || n == "thrust_max_pct") {
                     const double v = param_to_double(p);
                     if (!std::isfinite(v) || v < -100.0 || v > 100.0) {
                         return reject(n + ": 需在 [-100, 100] 内");
                     }
                     if (n == "thrust_min_pct") new_thrust_min = v;
                     else                       new_thrust_max = v;
                     thrust_touched = true;
                 } else if (n == "servo_max_deg") {
                     const double v = param_to_double(p);
                     if (!std::isfinite(v) || v <= 0.0 || v > 180.0) {
                         return reject(n + ": 需满足 (0, 180]");
                     }
                 } else if (n == "cmd_timeout_s") {
                     new_cmd_timeout = param_to_double(p);
                     if (!std::isfinite(new_cmd_timeout) || new_cmd_timeout <= 0.0) {
                         return reject(n + ": 必须为正数");
                     }
                 } else if (n == "control_rate_hz") {
                     new_rate = param_to_double(p);
                     if (!std::isfinite(new_rate) || new_rate <= 0.0 ||
                         new_rate > 1000.0) {
                         return reject(n + ": 需在 (0, 1000] 内");
                     }
                 } else if (is_dob) {
                     const double v = param_to_double(p);
                     if (!std::isfinite(v)) {
                         return reject(n + ": 必须为有限值");
                     }
                     if (dob_key == 0 && (v < 0.0 || v > 1000.0)) {
                         return reject(n + ": dob_bandwidth 需在 [0, 1000] 内");
                     }
                     if (dob_key == 1 && v <= 0.0) {
                         return reject(n + ": mass_eff 必须 > 0");
                     }
                     if (dob_key == 2 && v < 0.0) {
                         return reject(n + ": damp_eff 必须 >= 0");
                     }
                 } else if (n == "controller_mode") {
                     const int64_t v = param_to_int(p);
                     if (v < 1 || v > 3) {
                         return reject(n + ": 需为 1/2/3");
                     }
                 } else if (n == "imu_deg2rad") {
                     try { (void)p.as_bool(); } catch (const std::exception &) {
                         return reject(n + ": 需要 bool 类型");
                     }
                 } else if (n == "servo_cmd_hz") {
                     const double v = param_to_double(p);
                     if (!std::isfinite(v) || v < 0.0 || v > 200.0) {
                         return reject(n + ": 需在 [0, 200] 内");
                     }
                 } else if (n == "imu_yaw_offset_deg") {
                     if (!std::isfinite(param_to_double(p))) {
                         return reject(n + ": 必须为有限值");
                     }
                 } else if (n == "deadzone_pct") {
                     const double v = param_to_double(p);
                     if (!std::isfinite(v) || v < 0.0 || v > 100.0) {
                         return reject(n + ": 需在 [0, 100] 内");
                     }
                 } else if (n == "thrust_limits") {
                     const std::vector<double> v = p.as_double_array();
                     if (v.size() != 6U) {
                         return reject(n + ": 必须恰好 6 个元素");
                     }
                     if (!std::all_of(v.begin(), v.end(),
                             [](double x) { return std::isfinite(x) && x > 0.0 &&
                                                   x <= 1e6; })) {
                         return reject(n + ": 元素需在 (0, 1e6] 内且有限");
                     }
                 } else if (n == "mode_pid_enable_value" ||
                            n == "mode_pid_disable_value" ||
                            n == "mode_keyboard_enable_value" ||
                            n == "mode_keyboard_disable_value") {
                     const int64_t v = param_to_int(p);
                     if (v < 0 || v > 255) return reject(n + ": 需在 [0, 255] 内");
                     const size_t slot = (n == "mode_pid_enable_value")      ? 0U :
                                         (n == "mode_pid_disable_value")     ? 1U :
                                         (n == "mode_keyboard_enable_value") ? 2U : 3U;
                     new_modes[slot] = v;
                     modes_touched = true;
                 } else if (is_ff_param_name(n)) {
                     if (!std::isfinite(param_to_double(p))) {
                         return reject(n + ": 必须为有限值");
                     }
                 } else if (n.rfind("enable_", 0) == 0) {
                     (void)p.as_bool();  // 仅做类型检查
                 } else if (n == "use_sim_time") {
                     try { (void)p.as_bool(); } catch (const std::exception &) {
                         return reject(n + ": 需要 bool 类型");
                     }
                 } else if (n == "mode_topic" || n == "remote_topic") {
                     (void)p.as_string();  // 接受, 下次 configure 生效
                 } else {
                     std::string dof, key;
                     if (!split_pid_param_name(n, dof, key)) {
                         return reject("参数 " + n + " 不支持在线修改");
                     }
                     const double v = param_to_double(p);
                     if (!std::isfinite(v)) return reject(n + ": 必须为有限值");
                     if (key == "max_i" && v < 0.0) {
                         return reject(n + ": 必须 >= 0");
                     }
                     if (key == "max_out" && v <= 0.0) {
                         return reject(n + ": 必须 > 0");
                     }
                 }
             } catch (const std::exception & e) {
                 return reject(n + ": " + e.what());
             }
             accepted.push_back(p);
         }

         // 交叉约束 (成对/成组参数)
         if (thrust_touched && new_thrust_min >= new_thrust_max) {
             return reject("thrust_min_pct 必须小于 thrust_max_pct");
         }
         if (modes_touched) {
             std::array<int64_t, 4> sorted_modes = new_modes;
             std::sort(sorted_modes.begin(), sorted_modes.end());
             if (std::adjacent_find(sorted_modes.begin(), sorted_modes.end()) !=
                 sorted_modes.end()) {
                 return reject("四个模式命令值必须互不相同");
             }
         }

         // ---- 阶段 2: 应用 ----
         for (const rclcpp::Parameter & p : accepted) {
             const std::string & n = p.get_name();
             try {
                 int dob_row = -1;
                 int dob_key = -1;
                 const bool is_dob = split_dob_param_name(n, dob_row, dob_key);
                 if (n == "pid_deriv_alpha") {
                     pid_deriv_alpha_ = new_alpha;
                     for (PidController * c : {&pid_vx_, &pid_vy_, &pid_depth_,
                                               &pid_yaw_, &pid_pitch_, &pid_roll_}) {
                         c->alpha = pid_deriv_alpha_;
                     }
                 } else if (n == "alloc_lambda") {
                     alloc_lambda_ = new_lambda;
                 } else if (n == "alloc_matrix") {
                     alloc_vec_ = p.as_double_array();
                 } else if (n == "thrust_min_pct") {
                     thrust_min_pct_ = new_thrust_min;
                 } else if (n == "thrust_max_pct") {
                     thrust_max_pct_ = new_thrust_max;
                 } else if (n == "servo_max_deg") {
                     servo_max_deg_ = param_to_double(p);
                 } else if (n == "cmd_timeout_s") {
                     cmd_timeout_s_ = new_cmd_timeout;
                 } else if (n == "control_rate_hz") {
                     control_period_s_ = 1.0 / new_rate;
                 } else if (n == "mode_pid_enable_value") {
                     pid_enable_value_ = static_cast<uint8_t>(new_modes[0]);
                 } else if (n == "mode_pid_disable_value") {
                     pid_disable_value_ = static_cast<uint8_t>(new_modes[1]);
                 } else if (n == "mode_keyboard_enable_value") {
                     keyboard_enable_value_ = static_cast<uint8_t>(new_modes[2]);
                 } else if (n == "mode_keyboard_disable_value") {
                     keyboard_disable_value_ = static_cast<uint8_t>(new_modes[3]);
                 } else if (is_ff_param_name(n)) {
                     set_ff_param(n, param_to_double(p));
                 } else if (n == "enable_surge")  { en_surge_  = p.as_bool(); }
                 else if (n == "enable_sway")     { en_sway_   = p.as_bool(); }
                 else if (n == "enable_depth")    { en_depth_  = p.as_bool(); }
                 else if (n == "enable_yaw")      { en_yaw_    = p.as_bool(); }
                 else if (n == "enable_pitch")    { en_pitch_  = p.as_bool(); }
                 else if (n == "enable_roll")     { en_roll_   = p.as_bool(); }
                 else if (n == "enable_ff")       { en_ff_     = p.as_bool(); }
                 else if (n == "enable_pid")      { en_pid_    = p.as_bool(); }
                 else if (n == "enable_servo")    { en_servo_  = p.as_bool(); }
                 else if (is_dob) {
                     const double v = param_to_double(p);
                     if (dob_key == 0)      dob_[dob_row].bandwidth = v;
                     else if (dob_key == 1) dob_[dob_row].mass_eff  = v;
                     else                   dob_[dob_row].damp_eff  = v;
                     dob_[dob_row].reset();  // 模型系数变更: 清观测器状态避免突变
                 } else if (n == "controller_mode") {
                     controller_mode_ = static_cast<int>(param_to_int(p));
                 } else if (n == "imu_deg2rad") {
                     imu_deg2rad_ = p.as_bool();
                 } else if (n == "servo_cmd_hz") {
                     servo_cmd_hz_ = param_to_double(p);
                 } else if (n == "imu_yaw_offset_deg") {
                     imu_yaw_offset_ = param_to_double(p) * M_PI / 180.0;
                 } else if (n == "deadzone_pct") {
                     deadzone_pct_ = param_to_double(p);
                 } else if (n == "thrust_limits") {
                     const std::vector<double> v = p.as_double_array();
                     for (size_t i = 0; i < 6U; ++i) thrust_limits_[i] = v[i];
                 }
                 else if (n == "use_sim_time") {
                     // ROS2 通用参数: 接受但不参与运动控制 (仅便于整组 param load)
                 }
                 else if (n == "mode_topic" || n == "remote_topic") {
                     RCLCPP_WARN(get_logger(),
                         "[MC] 在线调参: %s 将在下次 configure 时生效", n.c_str());
                 } else {
                     std::string dof, key;
                     if (split_pid_param_name(n, dof, key)) {
                         set_pid_param_value(dof, key, param_to_double(p));
                     }
                 }
             } catch (const std::exception & e) {
                 RCLCPP_ERROR(get_logger(), "[MC] 在线调参: %s 应用失败: %s",
                              n.c_str(), e.what());
                 continue;
             }
             RCLCPP_INFO(get_logger(), "[MC] 在线调参: %s = %s",
                         n.c_str(), p.value_to_string().c_str());
         }

         // 控制频率变化时重建定时器 (配置完成后才需要)
         if (std::any_of(accepted.begin(), accepted.end(),
                 [](const rclcpp::Parameter & p) {
                     return p.get_name() == "control_rate_hz";
                 })) {
             restart_control_timer();
         }

         rcl_interfaces::msg::SetParametersResult result;
         result.successful = true;
         result.reason = "ok";
         return result;
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
     double servo_max_deg_   = 45.0;
     rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_{};
     double pid_deriv_alpha_ = 0.7;   // D 项一阶低通滤波系数 (原固定 0.7)
     double alloc_lambda_    = 0.01;  // 分配阻尼因子 (原固定 0.01)
     // -- 控制律 (python 版移植) --
     int controller_mode_ = 2;        // 1=纯PID 2=FF+PID 3=FF+PID+DOB
     double deadzone_pct_ = 3.0;      // 分配死区 (% of u_max)
     std::array<double, 6> thrust_limits_{441.0, 69.0, 69.0, 69.0, 69.0, 69.0};
                                        // 各推推力上限 (N) ⚠ 占位待实测
     std::array<DobController, 6> dob_{};          // DOB 状态 (行序 Fx..Mz)
     std::array<double, 6> tau_prev_real_{};       // 上一拍实际 wrench T·u (N)
     // -- 角速率估计 (姿态差分) --
     bool   imu_deg2rad_    = true;
     double imu_yaw_offset_ = 0.0;
     double servo_cmd_hz_ = 5.0;
     std::chrono::steady_clock::time_point last_servo_cmd_time_{};
     std::array<double, 4> last_tail_cmd_sent_{};
     std::array<double, 2> last_wing_cmd_sent_{};
     bool   servo_cmd_sent_once_ = false;
     bool att_initialized_ = false;
     double prev_yaw_ = 0.0;
     double prev_pitch_ = 0.0;
     double prev_roll_ = 0.0;
     std::chrono::steady_clock::time_point prev_att_time_{};
 
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
 
