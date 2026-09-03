# UVMS 航行器运动控制 —— 终版设计文档（评审稿 v0.1）

- 日期：2026-09-02（工作区文件时间线）
- 状态：评审稿，待评审决策点确认后进入实现
- 范围：航行器本体 6-DOF 运动控制（推进器为主，舵机预留），不含机械臂
- 关联文档：
  - `src/hal/docs/remote_control_instruction_format.md`（模式/遥控协议目标架构，本文档与其对齐并修正其漂移）
  - `src/hal/docs/dvl_acoustic_udp_topic_format.md`
  - 现状实现 A：`src/bsp/src/bsp_motioncontrol_node.cpp`（C++，PID+FF）
  - 现状实现 B：`bsp_py/uvms3_bsp_py/src/{bsp_motioncontrol_node.py, dof_controller.py, thrust_alloc.py}`（Python，FF+PID+DOB）

---

## 1. 背景与问题基线

现状存在两套并列实现，各有半成品问题，不能直接作为终版：

| # | 问题 | 位置 | 终版必须解决 |
| --- | --- | --- | --- |
| P1 | 两个功能包同名 `bsp`、节点同名 `bsp_motioncontrol_node`、争用同一话题 `/hal/thruster/cmd`，colcon/install 互相覆盖 | `src/bsp` vs `bsp_py/uvms3_bsp_py` | 收敛为单一包/单一节点实现 |
| P2 | 分配单位不自洽：无量纲 T 直接当百分比、无各推物理上限、饱和后盲截断不重分配 | 实现 A `allocate_thrust()` | 采用 N 单位分配 + 逐次截断（实现 B 方案） |
| P3 | 无扰动抑制（无 DOB），海流/浮力差/缆力全压给积分 | 实现 A | 引入 DOB（实现 B 方案） |
| P4 | 执行器故障反馈被订阅但未参与控制；DVL 有效性不参与门控 | 实现 A | 故障/传感器分级门控（见 §5） |
| P5 | enable 开关失效：关闭通道仍有 FF/DOB 输出 | 实现 B `_control_loop` | 关闭通道整体冻结（含内部状态） |
| P6 | 无生命周期、无 `/hal/modecontrol` 订阅、无与开环遥控节点互斥 | 实现 B | 模式所有权仲裁（见 §3.1） |
| P7 | `/app/motioncontrol` 无发布者、data[0] 空置、无协议文档 | 实现 B | 定义 app→BSP 指令协议（见 §3.2） |
| P8 | 文档/代码漂移：launch 参数名过时；`HalModeControl` 字段 `modecontrol_cmd` vs 文档 `command`；`HalRemoteControl` 字段 `tunnel*_para` vs 文档 `surge/sway/...` | 多处 | 统一文档并同步 msg 定义 |
| P9 | 可疑/占位物理参数：`roll.mass_eff=4.345` 量级异常；T 矩阵 Mx=0.1/My=0.3 系数无几何依据 | 实现 B yaml / 两版 T | 由几何与实测参数驱动（§8 索取清单） |
| P10 | `motion_control_validation.hpp` 编写完成但无调用点 | `src/bsp/include/hal/`、`src/hal/include/hal/` | configure 期接入参数校验 |
| P11 | install/build 产物过期且与 autostart 引用不一致 | workspace | 终版落地后全量重建验证 |

---

## 2. 目标架构与模块划分

设计原则：**算法内核与语言无关、节点壳与工程集成解耦、接口先行**。
语言选型（评审决策点 D1）不影响本节模块边界——C++ 与 Python 都按同样接口实现。

```text
上位机/地面站 UDP ──► bsp_comm_node ──► /hal/modecontrol(模式)
                                └──────► /hal/remotecontrol(手操通道)
app 自主层(规划/决策) ────────────► /app/motioncontrol(自主设定点)      [协议见 §3.2]

        ┌────────────────────────────────────────────────────┐
        │                 bsp_motioncontrol_node（终版）       │
        │                                                    │
        │  M1 输入适配与仲裁   模式状态机、所有权、RC映射/斜坡   │
        │  M2 状态与有效性     传感器缓存、pqr 估计、健康矩阵   │
        │  M3 控制内核         6×DOF FF+PID+DOB（纯函数）      │
        │  M4 推力分配         T 构造、截断伪逆、死区、N→%     │
        │  M5 安全与生命周期   使能/停机/看门狗/零推力          │
        │  M6 诊断            状态话题、录包、参数热更新        │
        └────────────────────────────────────────────────────┘
   ▲ 订阅            │ 发布                │ 生命周期
/hal/inertialnavi    /hal/thruster/cmd     startup_manager
/hal/dvl             /hal/servo/tail_cmd   (uvms_autostart)
/hal/depthsensor     /hal/servo/wing_cmd
/hal/mainthruster    /bsp/motioncontrol/status  [新增]
/hal/auxithruster
/hal/tailservo /hal/wingservo
```

模块接口（语言无关规格，供两版实现对照）：

| 模块 | 输入 | 输出 | 关键约束 |
| --- | --- | --- | --- |
| M1 | mode、RC 通道、/app 指令 | 6-DOF 目标设定点 + valid | 所有权互斥、超时清零 |
| M2 | 传感器原始消息 | 有效状态向量、健康标志 | 分级门控矩阵（§5） |
| M3 | 设定点、状态、上一拍实际 τ | 期望 wrench τ_des[6] | 纯函数、无 I/O、可单测 |
| M4 | τ_des、布局参数 | u_pct[6]（+ 饱和报告） | 单位=N、截断伪逆 |
| M5 | 模式/健康/生命周期事件 | 使能、零推力 | 全停条件有序判定（§7） |

---

## 3. 接口协议设计

### 3.1 模式与所有权仲裁（保留现有 1/2/3/4 语义，消除漂移）

以 `src/hal/docs/remote_control_instruction_format.md` 为权威：

| command | 常量 | 所有权 | 动作 |
| --- | --- | --- | --- |
| 1 | CMD_ENABLE_PID | 运动控制(手操 RC 闭环) | 复位 PID/DOB，清零设定点，开始消费 `/hal/remotecontrol` |
| 2 | CMD_DISABLE_PID | — | 退出 PID，发一次零推力，停发 |
| 3 | CMD_ENABLE_KEYBOARD | 开环遥控 | 开环节点接管 `/hal/thruster/cmd`，运动控制必须让出 |
| 4 | CMD_DISABLE_KEYBOARD | — | 退出开环，发一次零推力 |

**终版统一动作**：

1. 修正 `HalModeControl.msg`：字段定名 `command`（或保留 `modecontrol_cmd` 但全仓一致），并在 msg 中定义 1/2/3/4 常量，消除"代码默认值 vs launch 参数名 vs 文档字段名"三处不一致。
2. 模式值用 msg 常量 + 节点参数默认覆盖（保留现有参数化设计），launch 一律显式传参并只允许传同名参数（删除 `mode_pid_value`/`mode_keyboard_value` 这类幽灵参数）。
3. 终版运动控制节点必须订阅 `/hal/modecontrol`，并实现"收到 3（键盘接管）时自动让出 + 收到 1 时接管"的完整状态机（现 C++ 版已有此逻辑，直接复用为规格）。

### 3.2 自主指令协议 `/app/motioncontrol`（终版新定义，替代悬空草案）

沿用 `Float64MultiArray`（不改 msg 的前提下），**定死索引表**：

| index | 含义 | 单位/取值 | 缺省 |
| --- | --- | --- | --- |
| 0 | 指令类型/标志：0=速度+位置混合保持（现语义），1=…（预留） | int | 0 |
| 1 | vx（期望纵荡速度，体坐标） | m/s | 0 |
| 2 | vy（期望横荡速度，体坐标） | m/s | 0 |
| 3 | depth（期望深度，绝对，向下为正） | m | 当前深度(首帧拒绝) |
| 4 | yaw（期望艏向，绝对，需 wrap） | rad | 当前艏向(首帧拒绝) |
| 5 | pitch（预留，固定 0） | rad | 0 |
| 6 | roll（预留，固定 0） | rad | 0 |
| 7 | 心跳序号 seq（单调递增，看门狗使用） | int | 0 |

约束：每帧必须 ≥8 元素；非法值（NaN/±Inf）整帧拒绝；与模式仲裁的关系：
- `/app/motioncontrol` 是**自主设定点源**，仅在 CMD_ENABLE_PID 持有所有权期间生效（即 PID 模式同时支持 RC 手操与自主两条设定点来源，来源选择由 data[0] 或参数 `cmd_source` 决定，二选一评审 D4）；
- 现有 RC 手操映射（§5 的 hal 文档）保留为 `cmd_source=rc` 时的设定点来源。

### 3.3 消息与话题修正清单

| 项 | 现状 | 终版 |
| --- | --- | --- |
| `HalRemoteControl` 字段 | `tunnel1_para..tunnel6_para` | 建议改名 `surge/sway/heave/yaw/pitch/roll`（同步 hal 文档与 comm 节点解析），改动波及面小 |
| `HalModeControl` | `modecontrol_cmd` + 无常量 | `command` + 常量；或保留字段名仅加常量——二选一后全仓一致 |
| 新增 `/bsp/motioncontrol/status` | 无 | Float64MultiArray(32)：`[mode, enabled, t, yaw,pitch,roll, depth, vx,vy,vz, err_6, tau_6, dhat_6, u_pct_6, fault_bits]`（顺序以代码注释锁定）——调参/录包刚需 |
| 舵机输出 | 两版都发零位 | 终版预留 tail[4]/wing[2]，分配启用前保持周期零位 + `enable_servo=false` 默认 |

### 3.4 输出契约（不可变）

`/hal/thruster/cmd` = Float64MultiArray[6]，百分比 ±100，顺序 `[主推 CAN0x301, 辅推 ESC ID2..ID6]`（`hal_thruster_node` 已按此执行）。

---

## 4. 控制律规格（语言无关）

### 4.1 逐自由度控制律

每自由度一个控制器实例（surge/sway/depth/yaw/pitch/roll），执行：

```text
τ_i = τ_FF,i + τ_PID,i − d̂_i            （合成，mode 分级裁剪）
τ_FF,i = k_lin,i·ν_des + k_quad,i·ν_des·|ν_des|   （depth/pitch/roll 默认 0）
τ_PID,i = kp·e + ki·∫e·dt + kd·ė         （梯形积分、I 限幅、D 一阶低通 α=0.7）
d̂_i(k) = α·d̂_i(k−1) + (1−α)·[ τ_prev,i − m_eff,i·(ν_k−ν_k−1)/dt − d_eff,i·ν_k ]
α = exp(−L_i·dt)，L_i = dob_bandwidth（0 = 禁用 DOB）
e: surge/sway=速度误差；depth=位置误差；yaw/pitch/roll=角度误差(先 wrap)
ν_actual: surge/sway/vz（DVL）；yaw/pitch/roll=p,q,r（估计，见 4.3）
τ_prev,i：上一拍实际施加 wrench = T·u_cmd(N)（含饱和，饱和时即真实施加值）
```

要点（全部来自对两版实现的评审结论）：

1. **mode 分级**：`controller_mode`：1=纯PID，2=FF+PID，3=FF+PID+DOB；上实机按 1→2→3 顺序投用。
2. **使能语义修正（P5）**：`enable_xxx=false` 时该自由度整周期冻结——不计算 FF/PID/DOB、不更新任何内部状态、输出 0；DOB 的 `τ_prev` 相应取 0。全局开关 `enable_pid/enable_ff/enable_dob` 同语义。
3. **DOB 状态纪律**：α 在参数(带宽/dt)变更或 reset 时重算（修实现 B `_dob_alpha` 惰性计算问题）；reset 清 `d̂/∫e/差分缓存/α`。
4. **状态量纲**：τ 统一为 N / N·m；ν 统一 m/s / rad/s。PID 增益需在真实推力上限下整定（见 §6 与 §8），避免"误差 1 m → 需求 300 N > 物理 138 N"的必然饱和。
5. **设定点斜坡**：所有位置/速度设定点在节点内做一阶限速（RC 通道天然连续可不加，自主指令必须加：vx ≤0.5 m/s²、depth ≤0.5 m/s、yaw ≤30°/s 起步值，可参数化）；yaw 全程 wrap。
6. **浮力配平**：`buoyancy_trim` 作为 depth 通道静态前馈（FF 覆盖项），DOB 开环带宽足够后可逐步让 DOB 接管（可设 0 并观察 d̂_depth 收敛值反推配平量——这是免费的系统辨识）。
7. **抗积分饱和再加强**：除 I 限幅外，实现 B 的截断分配天然把"物理不可达的 τ"暴露为饱和报告（§6.3），可据此做有条件积分冻结（积分仅在对应推进器未饱和时增长）——v1 可作为增强项。

### 4.2 角速率估计（p,q,r）

- 姿态角来自 IMU 的 yaw/pitch/roll，频率可能远低于控制频率（注意控制循环内差分=按控制周期 50Hz 差分传感器上一次值，需按**到达时刻**差分，修实现 B 用控制周期近似的问题：记录每次姿态回调时间戳，估计器只用真实 dt 与新鲜样本）。
- 欧拉角速率 → 体坐标：用 J2_inv 精确变换（实现 B 已有），并加一阶低通（截止 ~8–15 Hz 可参数化 `rate_lpf_hz`）。
- 传感器节流时（IMU 低于控制频率），差分间隔取"最近两次有效姿态"，估值输出保持，并设"姿态新鲜度"标志，超过 `sensor_timeout_s` 视为失效。
- 若未来 IMU 消息提供角速度（msg 暂无），直接改用并废弃差分——协议预留。

### 4.3 控制频率与执行模型

- 固定 `control_rate_hz=50`（两版一致），wall timer；
- 周期内若关键输入无新样本，用缓存值 + 新鲜度判断（§5）；
- 参数热更新：注册 ROS2 参数事件回调，变更只在控制周期边界生效，禁止每周期 `get_parameter`（实现 B 现状，50Hz×数十次调用是纯浪费）。

---

## 5. 传感器有效性与失效降级

### 5.1 健康矩阵

| 自由度 | IMU | DVL | 深度计 | 失效动作 |
| --- | --- | --- | --- | --- |
| surge/sway | — | 必需 | — | DVL 失效 → 禁该两轴（或整体速度模式退出），不触发全停 |
| depth | — | 供 DOB(vz) | 必需 | 深度计失效 → 禁 depth 轴，垂推零位 |
| yaw/pitch/roll | 必需 | — | — | IMU 失效 → **全停**（姿态是所有轴的基准） |
| 全部 | — | — | — | `/hal/thruster/cmd` 无人订阅或推进器节点断连 → 零推力（依赖 hal 侧超时兜底，实现前需确认 `hal_thruster_node` 是否具备指令超时，若无则终版补：M5 持续监测话题连接） |

> 评审点 D5：DVL 失效时"禁 sway/surge 但保留深度/艏向保持"是否可接受，还是直接全停（现实现 B 是全停，现实现 A 是完全不查 DVL）。建议采用上表分级方案。

### 5.2 执行器故障

- `HalMainthruster.fault_status`、`HalAuxithruster.fault_status[i]`（含 `esc_status` 可展示）：
  - 主推故障 → 禁 surge（航向/深度/横荡可继续）；
  - 辅推故障 → 对应列从分配矩阵移除并在残余分配中降级（截断伪逆天然支持列剔除），故障列同时上报；
  - 多推故障 → 按剩余可达性判定全停（预计算：剩余推进器生成的 wrench 空间是否仍含 [Fx,Fz,Mz] 主任务，可在线用 T 子矩阵秩判定）。
- 阈值与确认：故障位需持续 N 拍（`fault_confirm_ticks`）才动作，防毛刺；动作后日志 + status 话题置位。

### 5.3 看门狗分层

| 层 | 条件 | 动作 |
| --- | --- | --- |
| 指令超时 | RC 或 /app 超过 `cmd_timeout_s`（1.0s） | 零推力 + 模式保持 + 复位积分 |
| 传感器超时 | 关键传感器新鲜度 > `sensor_timeout_s` | 按 5.1 表 |
| 心跳 | 自主指令 seq 不递增 | 视为指令超时同层 |

---

## 6. 推力分配规格

### 6.1 布局参数与 T 构造（替代硬编码 36 元素矩阵）

```text
推进器 i: 位置 r_i = [x,y,z]ᵢ（相对重心/浮心，m）
          方向 v_i = [vx,vy,vz]ᵢ（单位向量，指向正推力方向）
          u_max,i（正向最大推力 N）、u_max_rev,i（反向，常 < 正向）
          死区 u_dz,i（N 或 %）
T[:,i] = [ v_i ;  r_i × v_i ]          （6 维列）
```

- 符号/方向以台架实测为准（评审 D6）：单推正转 → 力方向 → 与 CAN 正指令对应关系锁定进配置，不许靠代码内猜。
- `alloc_matrix` 参数退化为"调试期直接覆盖"选项；正常配置由布局参数在 configure 时计算并校验（满秩、量纲检查）——`motion_control_validation.hpp` 的校验逻辑在此接入（P10），Python 侧同样保留等价校验。

### 6.2 分配算法（采用实现 B 规格）

```text
u = pinv(T) τ
repeat(≤6 次):
   饱和集 S = { i | |u_i| ≥ u_max,i }        （正反向分别用各自上限）
   若 S 空: break
   u_S = sign(u_S)·u_max,S                  （钉死）
   τ_rem = τ − T_S·u_S
   u_free = pinv(T_free)·τ_rem
   u = u_free ∪ u_S
u = clip(u, ±u_max)                          （符号对称，反向弱化建模后续增强）
u_cmd,i = 0 若 |u_i| < u_dz,i
τ_real = T·u_cmd                             （喂 DOB）
pct_i  = u_i / u_max,i · 100                 （按各推上限归一）
```

### 6.3 饱和报告与可达性

- 分配后残差 `τ_err = τ_des − T·u_cmd` 每周期计算并发布到 status（这是判"设定点是否物理可达"的唯一依据，也是调参指标：若常饱和 → 降增益/降任务速度而非加积分）。
- 建议 v1 增强：对垂推与侧推设置**效率低区**（低速正反转切换区），由死区 + 指令最小步进覆盖。

---

## 7. 安全与生命周期

1. 终版节点实现 ROS2 **LifecycleNode**（若语言评审选 Python：用 `rclpy_lifecycle.LifecycleNode` 提供等价接口，保证 startup_manager 分组不改）：
   - configure：建订阅/发布、布局计算、参数校验（失败 → FAILURE）；
   - activate：复位全部控制器，允许进入模式仲裁；
   - deactivate/shutdown/error：**无条件零推力一次 + 冻结输出**（现实现 A 逻辑作为规格）。
2. 模式所有权：CMD 2/4 与对方接管(1/3) 都触发本侧零推力 + 复位 + 停发（两 C++ 节点现有互斥逻辑原样进入规格）。
3. 全停条件有序判定（每周期，优先级从高到低）：节点未激活 → 收到 2/4 → 关键传感器失效 → 多推故障不可达 → 指令超时 → 其余告警不停机。
4. autostart：`uvms_autostart.launch.py` 的 bsp 分组（comm → remote+motion）保持；终版落地后 **install 全量重建并 smoke test**（P11）。

---

## 8. 参数化与 YAML 模板（终版草案）

```yaml
bsp_motioncontrol_node:
  ros__parameters:
    control_rate_hz: 50.0
    cmd_timeout_s: 1.0
    sensor_timeout_s: 0.5
    controller_mode: 1            # 1=纯PID 2=FF+PID 3=FF+PID+DOB（默认从 1 起步）
    cmd_source: rc                # rc | app（评审 D4）
    enable: {surge: true, sway: true, depth: true, yaw: true, pitch: true, roll: true,
             pid: true, ff: true, dob: false}
    # RC 映射（heave→depth 等）—— 照 hal 文档 §5，唯一改动: 全部参数可配
    ramp: {vx_max: 0.5, vy_max: 0.25, depth_max: 0.5, yaw_max_deg_s: 30.0}
    rate_lpf_hz: 10.0
    fault_confirm_ticks: 5

    surge: {kp: ■, ki: ■, kd: ■, max_i: ■, max_out: ■, ff_linear: ■, ff_quadratic: ■,
            dob_bandwidth: ○, mass_eff: ■, damp_eff: ■}
    # sway/depth/yaw/pitch/roll 同构；depth 另含 buoyancy_trim: ■
    # （■=需实测/模型取值，○=可先占位——见 §9 索取清单）

    thrusters:
      layout_defined: true       # false 时启用旧 alloc_matrix 覆盖路径(仅调试)
      # 每推: {id, can_id, r: [x,y,z], v: [vx,vy,vz], u_max_fwd: ■, u_max_rev: ■}
      # 或直接列 36 元素 alloc_matrix（调试覆盖）
    thrust_deadzone_pct: 3.0
```

---

## 9. 物理参数索取清单（需要你们从模型/文档提供）

按重要性排序，交付形式建议：SolidWorks 导出表 / 实测记录 / 既有参数表，任一均可，单位务必注明。

| 组 | 参数 | 用途 | 单位 |
| --- | --- | --- | --- |
| G1 推进器布局 | 6 推的位置坐标 r_i（相对载体坐标系原点/重心）与推力轴线方向 v_i；主推/垂推/侧推的安装角 | T 矩阵几何构造 | m、单位向量 |
| G2 推力能力 | 各推正/反向最大推力（额定电压下）、推力–指令百分比曲线或 rpm–力曲线、死区/启动力 | u_max、N→%、分配死区 | N |
| G3 载体惯性 | 质量、重心位置、惯量 Ixx/Iyy/Izz（及产品惯量积）、浮心位置与浮力、配平压载量 | mass_eff（平动=质量+附加质量、转动=惯量）、buoyancy_trim、roll 4.345 疑点核对 | kg、kg·m²、m |
| G4 阻尼 | 各轴线性/二次阻尼系数（CFD 或自由衰减试验），或允许我按经验初值并设计辨识试验 | FF/DOB 的 k_lin/k_quad、damp_eff | N·s/m、N·s²/m²、N·m·s、N·m·s² |
| G5 传感器 | IMU 输出频率与安装姿态、DVL 有效工作范围/底跟踪、深度计量程精度；深度计零点/安装高度偏移 | 状态估计、门控阈值、深度真值 | Hz、m |
| G6 遥控器 | 6 通道与杆的对应（哪根杆是 surge/sway/heave/yaw），通道中位/死区实测值 | RC 映射表 | 原始量 |
| G7 电机/推进器硬件 | 主推 CAN0x301 与辅推 ESC ID2–ID6 的正指令旋转方向与推力方向关系 | 符号约定（评审 D6） | — |

若部分参数暂无（尤其 G4），设计上全部可先取占位值并按 `controller_mode` 1→2→3 顺序现场整定，不影响代码结构落地。

---

## 10. 测试与验证计划

| 阶段 | 内容 | 通过判据 |
| --- | --- | --- |
| T1 单元 | PID/FF/DOB 数值行为、分配器饱和/截断/列剔除、yaw wrap、有效性矩阵 | 现有 11 个 Python 测试迁移 + 新增用例全绿 |
| T2 内核在环 | 用载体 6-DOF 仿真（含 G1–G4 参数）跑设定点/扰动场景 | 各轴收敛、无极限环、DOB 稳态估计合理 |
| T3 话题 mock | `sensor_mock_node`/`manual_mock_test` 扩展 + `mock_ground_station.py`（hal/test 已有）驱动模式切换 | 模式所有权、零推力、看门狗行为正确 |
| T4 台架 | `hal/launch/test_single_motor.launch.py` 单推正反符号验证（G7） | 符号表锁定进配置 |
| T5 系泊分级 | mode 1→2→3；深度/艏向阶跃与扰动 | 稳态误差、超调、饱和占比（status 话题录包评估） |
| T6 航行 | 直线、转向、悬停、故障注入 | 按 §5.2 表降级正确 |

---

## 11. 评审决策点清单

| ID | 决策 | 建议 |
| --- | --- | --- |
| D1 | 终版语言/基座：C++（复用现成 lifecycle/autostart/仲裁壳）还是 Python（迭代快，需补工程件） | 我建议 C++ 壳 + 移植 §4/§6 算法；若你们主控跑 python 生态则选 Python 并补生命周期。**文档不锁死，接口已解耦。** |
| D2 | 双 `bsp` 包如何收敛（删一留一 / 分名 bsp_core / 目录重组） | 保留唯一包名 `bsp`，另一仓库归档 |
| D3 | `/hal/remotecontrol`、`/hal/modecontrol` 字段改名 vs 仅加常量（波及 comm 与地面站协议） | 与上位机协议同步改，一次性 |
| D4 | PID 模式下设定点来源：RC 与 /app 并存/切换规则 | 先 `cmd_source` 参数二选一，互操作后续 |
| D5 | DVL 失效动作：分级降级 vs 全停 | 建议分级（§5.1） |
| D6 | 推力正负符号以台架实测为准 | 接受 |
| D7 | depth 的 max_out、各 PID max_out 是否按物理上限重新整定 | 按 G2 实测后整定 |

---

## 附录 A：两版现状对照速查（评审用）

| 维度 | 实现 A (C++) | 实现 B (Python) | 终版取 |
| --- | --- | --- | --- |
| 控制律 | PID+FF | PID+FF+DOB | B 算法（§4） |
| 分配 | DLS+盲截断、%量纲 | 截断伪逆、N 量纲 | B（§6） |
| enable | 真冻结 | 失效 | A 语义 |
| 模式仲裁 | 完整 | 无 | A（§3.1） |
| 生命周期 | 完整 | 无 | A 壳（§7） |
| 门控 | imu/depth，忽略故障与 DVL | imu/dvl/depth+故障全停 | 分级新表（§5） |
| 测试 | 无 | 11 单测 + mock | 迁移并扩展（§10） |
| 物理量纲 | 混 | 一致 | B |
| 参数校验 | 头文件未接线 | 无 | 校验接入 configure |

---

## 附录 B：两套运控实现差距对照与合并映射（v0.2 更新）

> 背景：实机体系已确定为 C++ 包 `bsp`（`src/bsp`），已为其加入在线调参（参数校验回调）。
> 注：启动参数随包基线/外置覆盖为 autostart 负责人待办需求，见 `handoff_autostart_change.md`；
> 全量参数参考文件见 `bsp_motioncontrol_reference.yaml`。
> `bsp_py/uvms3_bsp_py` 的运控内容为算法候选（DOB/分配），合并目标是**单节点、接口不变**。

| 维度 | C++ 节点 `bsp_motioncontrol_node.cpp`（实机现行） | bsp_py 运控（python 三件套 + yaml） | 合并取舍 |
| --- | --- | --- | --- |
| 节点形态 | LifecycleNode，autostart/启动管理器集成 | 普通 Node，无生命周期 | **保留 C++ 壳**（已接好体系） |
| 指令来源 | `/hal/remotecontrol` 摇杆 + `/hal/modecontrol` 1/2/3/4 仲裁 | `/app/motioncontrol`（data[1..4]，无发布者） | 两者共存：`cmd_source=rc/app`，仲裁保留 |
| 控制律 | PID + 稳态阻尼前馈 + 浮力常数 | 逐 DOF FF+PID+**DOB**，mode 1/2/3 分级 | **移植 DOB**（含 mode 分级） |
| 角速度利用 | 仅 yaw 前馈用目标差分（噪声大） | p/q/r 体坐标估计（J2_inv + 差分），喂 DOB/阻尼 | **移植 p/q/r 估计**供角 DOF |
| 使能语义 | 关闭通道真冻结（输出 0 且不更新） | 关闭通道 FF/DOB 仍输出（bug） | 保留 C++ 语义（正确） |
| 推力分配 | 6×6 DLS 伪逆、无量纲当 %、盲截断 | 逐次截断伪逆、`u_max=[441,69×5]N`、死区、N→% | **移植 python 方案**（单位一致） |
| 传感器门控 | imu+depth；DVL 失效不查、推进器故障忽略 | imu+dvl+depth + 主/辅推故障即停 | 升级：健康矩阵分级（§5.1，D5 评审） |
| 深度通道 | 位置环，仅深度反馈 | 位置环 + vz 进 DOB | 合并后：位置环 + DOB(vz) |
| 在线调参 | ✅ 已实现（两阶段校验回调 + 日志 + YAML 基线） | 每周期 get_parameter（低效但能生效） | 保留 C++ 回调机制 |
| 参数配置 | 无文件 → 已补 `config/bsp_motioncontrol.yaml` | 完整 yaml（含 mass_eff/damp_eff/dob 系数） | 扩充 C++ yaml（移植参数名） |
| 测试 | 无单测，冒烟验证 | 11 个单测全绿 + mock 集成脚本 | 单测迁移到 C++ 算法层（或保留 python 对照测试） |
| 物理参数 | T 无量纲占位、阻尼值未标定 | mass_eff/damp_eff 有值（roll=4.345 存疑） | 统一按 §9 索取清单实测替换 |

### 合并实施顺序（全部只动 `bsp_motioncontrol_node` 一个节点 + 自身 yaml）

1. **M-alloc 分配器升级**：新增 `thrust_limits[6]`(N)、逐次截断 + 死区 + N→%，DLS 退化为子解器；饱和残差进状态发布。
2. **M-dob 控制律升级**：每 DOF 新增 `dob_bandwidth/mass_eff/damp_eff` 参数与 d̂ 状态；`controller_mode` 1/2/3；p/q/r 估计器；冻结语义不变。
3. **M-safe 门控升级**：DVL/故障按健康矩阵分级介入（D5 定案后实现）。
4. **M-cmd 自主指令**：订阅 `/app/motioncontrol`，`cmd_source` 切换，协议按 §3.2。
5. **M-status 状态发布**：`/bsp/motioncontrol/status`（targets/state/τ/d̂/u/饱和/故障位），调参与录包刚需。
6. **M-params 参数合并**：新参数全部并入现有在线回调 + yaml 基线，保持两阶段校验。

每一步可独立编译、mock 验证、系泊分级测试（mode 1→2→3），不依赖其余步骤。
