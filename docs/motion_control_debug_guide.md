# 运动控制：实机运行与在线调参操作手册（v1.0）

> 适用对象：`bsp_motioncontrol_node`（C++，控制律 FF+PID+DOB，指令来源为遥控器/键盘）
> 配套文档：`motion_control_design.md`（设计细节与参数索取清单 §9）
> 使用前请先通读本手册，尤其 **第 0 节安全须知**。

---

## 0. 安全须知（先读）

- 本节点是闭环控制器，**切到 PID 模式（模式 1）后推进器由它接管**，任何误操作都直接作用到艇上。
- 出问题时的三把"急停"（按优先级）：
  1. 地面站发送 **模式 2（关闭 PID）** 或 **模式 4（关闭键盘）**——节点退出并输出一次零推力；
  2. 地面站切 **模式 3（键盘开环）**——开环节点接管（此时艇按你的摇杆/键盘走，也请立刻回中）；
  3. **拔遥控器/断通信 1 秒**——指令看门狗超时（`cmd_timeout_s`，默认 1.0 s）自动零推力。
- 任一关键传感器无效（IMU / DVL / 深度计 `connection_status != 1`）或推进器报故障 → **节点自动零推力停机**（D5 决策，全停，不能降级运行）。这不是故障，是保护，先查传感器。
- 在线改参数即刻生效（下一控制周期 ≈20 ms）。**改增益/限幅前先把艇稳定住或退回模式 2/零推力**，禁止在剧烈运动时改大 kp。
- 首次使用某组参数前，先在系泊/浅水验证，全程有人盯水面与遥控。

---

## 1. 系统结构与启动

### 1.1 参与运动控制的节点与话题

```text
上位机/地面站 UDP
   └─► bsp_comm_node ──► /hal/modecontrol      模式命令 1/2/3/4
                  └───► /hal/remotecontrol     遥控通道 (353~1695, 中位1024)

传感器(HAL驱动) ──► /hal/inertialnavi  姿态 yaw/pitch/roll + connection_status
              ──► /hal/dvl            体速度 vx/vy/vz + connection_status
              ──► /hal/depthsensor    深度 depth_avg + connection_status
执行器反馈   ──► /hal/mainthruster    fault_status
              ──► /hal/auxithruster   fault_status[5]

bsp_motioncontrol_node (本节点, 闭环PID/FF/DOB)
   └─► /hal/thruster/cmd   Float64MultiArray[6], 百分比 ±100
                           [主推0x301, 辅推ID2,ID3,ID4,ID5,ID6]
   └─► /hal/servo/tail_cmd, /hal/servo/wing_cmd  (当前周期发零位)

bsp_remotecontrol_node (开环键盘/摇杆直驱, 与PID模式互斥)
   └─► /hal/thruster/cmd   (只有模式3生效时才发)
```

模式命令协议（`/hal/modecontrol`）：

| 值 | 含义 | 谁接管推进器 |
|---|---|---|
| 1 | 开启 PID 闭环 | bsp_motioncontrol_node |
| 2 | 关闭 PID | 无人（零推力） |
| 3 | 开启键盘开环 | bsp_remotecontrol_node |
| 4 | 关闭键盘开环 | 无人（零推力） |

### 1.2 正常启动流程（自动）

```bash
cd ~/UVMS_WS
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bsp uvms_autostart.launch.py        # 车上正常部署即由此启动
```

autostart 会：上电（12/24/72V 分组）→ 启动 hal 传感器/推进器节点 → `bsp_comm/remotecontrol/motioncontrol` → startup_manager 自动把各 lifecycle 节点 configure+activate。

启动后确认（5 条命令全过再下水调试）：

```bash
ros2 node list | grep -E "bsp_motion|bsp_remote|bsp_comm|hal_thruster"   # 都在
ros2 lifecycle get /bsp_motioncontrol_node        # 期望 active
ros2 topic hz /hal/inertialnavi                   # 期望 >0（IMU）
ros2 topic hz /hal/dvl                            # 期望 >0（DVL, D5门控依赖它!）
ros2 topic hz /hal/depthsensor                    # 期望 >0
```

> D5 提醒：DVL 无效时 PID 模式直接零推力，调深度的前提是 DVL 有效（即使深度环不用速度，门控仍要求它在线）。

### 1.3 手动启动（仅调试用，不经过 autostart）

```bash
ros2 run bsp bsp_motioncontrol_node --ros-args \
  --params-file install/bsp/share/bsp/config/bsp_motioncontrol.yaml &
sleep 3
ros2 lifecycle set /bsp_motioncontrol_node configure
ros2 lifecycle set /bsp_motioncontrol_node activate
```

### 1.4 控制律与三档模式（controller_mode）

```
τ = τ_PID                      controller_mode = 1  (纯PID, 最保守, 调PID用这档)
τ = τ_FF + τ_PID               controller_mode = 2  (默认, 等同老版FF+PID)
τ = τ_FF + τ_PID − d̂           controller_mode = 3  (加扰动观测器DOB, 补偿海流/浮力差/缆力)
```

DOB 公式（每自由度独立）：`d̂ = α·d̂₋ + (1−α)·[τ_prev − m_eff·ν̇ − d_eff·ν]`，α = exp(−带宽·dt)。
`*.dob_bandwidth = 0` 表示该通道禁用 DOB。

三档只影响"控制律组合"，**不改变**模式 1/2/3/4 的"谁接管"逻辑（那是 `/hal/modecontrol` 的事）。两者别混淆：
- `/hal/modecontrol` 1/2/3/4 → 地面站切"人/机"；
- `controller_mode` 1/2/3 → 机控时用哪档控制律。

---

## 2. 参数体系速查

### 2.1 参数基线（开机默认值）现状说明

- 当前节点的"开机默认参数"= 代码内 declare 的默认值（即下表数值）；
- 完整的参数基线参考文件（全量 yaml）保留在 `docs/bsp_motioncontrol_reference.yaml`，
  供评审/制作随包配置文件用；
- "由 autostart 启动时加载随包 yaml、以及是否支持外置覆盖"是 **autostart 负责人的需求**
  （`docs/handoff_autostart_change.md`），落地前开机值以代码默认值为准；
- **运行期在线调参不受以上任何影响**：主控/开发机上节点运行中直接用 param set/load 即可。

> 若将来随包基线落地：改完 yaml 需重新安装 bsp 才生效：
> `colcon build --packages-select bsp`（20s 左右，只动 bsp，不需要断整船电），
> 然后重启 launch 或等下次开机。

### 2.2 参数分组

| 分组 | 参数 | 说明 |
|---|---|---|
| 运行 | `control_rate_hz` (50), `cmd_timeout_s` (1.0), `controller_mode` (2), `pid_deriv_alpha`(0.7), `alloc_lambda`(0.01), `deadzone_pct`(3.0) | 运行期可改；改频率会重建定时器 |
| PID | `pid_vx/vy/depth/yaw/pitch/roll .kp/.ki/.kd/.max_i/.max_out` | 每自由度独立 |
| 前馈 | `ff.drag_linear_x / quadratic_x / _y / _z / _yaw…`, `ff.buoyancy_trim` | 目前仅 surge(x)/sway(y) 用阻尼前馈, depth 用浮力配平 |
| DOB | `surge|sway|depth|yaw|pitch|roll .dob_bandwidth/.mass_eff/.damp_eff` | 带宽 0=关；数值 ⚠ 待实测替换 |
| 分配 | `alloc_matrix`(36元素), `thrust_limits`([441,69,69,69,69,69] N ⚠) | τ=T·u, u 单位 N, 再换算百分比 |
| 限幅 | `thrust_max_pct/min_pct`(±100), `servo_max_deg`(45) | 输出兜底 |
| 使能 | `enable_surge/sway/depth/yaw/pitch/roll/ff/pid/servo` | false=该通道完全冻结 |
| 模式映射 | `mode_pid_enable_value`(1)… `mode_keyboard_disable_value`(4) | 与上位机协议一致即可, 一般不动 |

### 2.3 在线调参命令

```bash
ros2 param list   /bsp_motioncontrol_node                       # 全部参数
ros2 param get    /bsp_motioncontrol_node pid_depth.kp          # 查单个
ros2 param set    /bsp_motioncontrol_node pid_depth.kp 300.0    # 改(立即生效)
ros2 param set    /bsp_motioncontrol_node controller_mode 3     # 切控制律档
ros2 param set    /bsp_motioncontrol_node enable_yaw false      # 临时关某通道
```

**必须注意：**
- double 参数要带小数点：`300.0` 可以，`300` 会被 ROS2 参数服务拒绝（显示类型错误）；
- 非法值会被节点拒绝并给出原因（日志与命令输出可见），例如：`kp<0`、`max_out<=0`、`controller_mode=5`、`thrust_limits` 长度≠6、DOB `mass_eff<=0`、4 个模式值重复；
- 每次成功修改节点日志会打 `[MC] 在线调参: 参数名 = 值`，修改后回看日志确认；
- 改 `control_rate_hz` 会重建定时器（日志有提示），DOB 的滤波系数随之自动重建；
- **在线改的值只在本次开机有效**，固化流程见 §5。

### 2.4 整组参数一键应用（预设热切换，最推荐）

> 场景：想把"一组参数"（如整套 yaw 增益 + DOB 系数）在**节点运行中**整体换掉，
> 不用重编、不用重启、其它节点照常工作。做法＝改文件 → `ros2 param load`。

```bash
# 1) 制作预设文件（以完整基线为模板，只改想调的值）
cd ~/auv_tuning
cp bsp_motioncontrol.yaml preset_yaw_A.yaml
#    用编辑器修改 preset_yaw_A.yaml 里的数值（注意 double 带小数点）
#    然后把文件顶部的键改成带斜杠的完整名：
#        bsp_motioncontrol_node:  →  /bsp_motioncontrol_node:

# 2) 运行中整体应用（逐项校验、逐条日志、失败项给出原因）
ros2 param load /bsp_motioncontrol_node ~/auv_tuning/preset_yaw_A.yaml
#    看到 "Set parameter xxx successful: ok" 即为生效

# 3) 验证
ros2 param get /bsp_motioncontrol_node pid_yaw.kp
```

- 每个预设文件 = 一份完整配置副本，可存多组（浅水/深水、模式2/模式3、粗调/精调），现场几秒钟切换；
- `ros2 param dump` 只能导出"与默认值不同"的参数，**不适合做预设模板**；预设请用本方法（§5 的 dump 只用于存档/回溯）。

> **免重编总览（三种粒度）**：
> 1. 改单个参数 → `ros2 param set`（§2.3）；
> 2. 改一组参数 → 编辑预设文件 + `ros2 param load`（§2.4）；
> 3. 改"下次开机基线" → 更新随包基线 `config/bsp_motioncontrol.yaml`（需重新安装/编译 bsp）
>    或部署外置覆盖文件（机制由 autostart 负责人定，见 `docs/handoff_autostart_change.md`；
>    **注意：本开发机上的 `~/auv_tuning/` 文件不会自动出现在主控上**）。
>    在线调参（1、2）与文件无关，在主控/开发机均可直接用。

---

## 3. 实操流程：系泊调参（第一次/每次改参数后）

> 原则：**小步、单项、稳定了再动下一项**。每次只改 1 个参数、观察 10~30 s、记录。
> 建议从 mode1（纯PID）起步逐档升：mode1 调稳 → mode2 看前馈是否改善 → mode3 开 DOB。

### 3.1 阶段 0：准备

1. 确认 §1.2 五条命令全过；传感器全部 `connection_status=1`（`ros2 topic echo /hal/inertialnavi --once` 等）；
2. 艇系泊/浅水、周围无人、遥控器满电、急停可用；
3. 摇杆全部回中，先让地面站发**模式 3**（键盘开环）做推进器方向检查：
   - 逐通道轻推，确认每个推进器正反转与预期一致、回中即停；发现方向反了 → **记录并停手**（这是符号/安装问题，不是调参能解决的，参考 §7 第 8 条）；
4. 发模式 4 关闭键盘，确认艇静水零速。

### 3.2 阶段 1：PID 模式静态安全确认（mode1）

1. 在线把 `controller_mode` 置 1（纯 PID，最保守）；
2. 只留最小通道集：先只调 yaw 或 depth 之一，其余 enable 置 false（例如 `enable_depth false enable_surge false enable_sway false`，先练 **yaw**）；
3. 把 yaw 摇杆拨到"期望艏向"的位置（ch4 是**绝对航向**：中位=0°、满杆≈±180°，切 PID 前务必先对准目标航向，否则一切入艇就转）；
4. 地面站发**模式 1**（开启 PID）：观察 10 s——艇应缓慢转向目标并稳住；
5. 若不稳/振荡：把 kp 减半再试；若太肉：加 kd；稳定但有残余角差：加 ki（一次加 1/2~1/4 原值）。

### 3.3 阶段 2：逐自由度整定顺序（建议）

顺序建议（从最稳、耦合最小的开始）：**roll → pitch → yaw → depth → sway → surge**。

每自由度套路（以 yaw 为例）：

| 步骤 | 操作 | 观察/判据 |
|---|---|---|
| 1 | `kp` 从小到大（如 100→200→400） | 出现持续振荡时**回退到上次稳定值**（通常取临界值的 40~60%） |
| 2 | 调 `kd` 抑制超调 | 接近目标时不再来回冲过 |
| 3 | 加 `ki`（先小，如 5）消稳态差 | 静止后艏向不再缓慢漂移 |
| 4 | 限幅 `max_i/max_out` | 保证任何情况下输出可被物理执行 |

观察手段：
```bash
ros2 topic echo /hal/thruster/cmd        # 看六个百分比: 是否持续 ±100(饱和)? 高频抖?
# 结合艇的实际姿态/深度反应判断, 记录现象
```

**判读要点：**
- 某通道输出**长期顶在 ±100%** → 该轴需求超过物理推力：降 kp/ki 或降低任务量（参考 §7 第 6 条），并检查 `thrust_limits` 是否偏小；
- 高频抖振 → kp 过大 / 传感器噪声（D 项滤波系数 `pid_deriv_alpha` 可加大）；
- 有稳态误差 → 加 ki（depth 可先调 `ff.buoyancy_trim` 抵消静态浮力差，比 ki 更"干净"）。

### 3.4 阶段 3：depth 专项

1. `enable_yaw false enable_depth true`（或同时开 yaw 保持航向，取决于你的测试目标）；
2. ch3 是**绝对深度 0~10 m**（中位≈5 m），先把目标深度对准当前深度附近再切模式 1；
3. depth 误差大时垂推容易饱和（两路垂推合计上限仅 2×69 N），**深度阶跃别给太大**（建议 ≤1 m 起步）；
4. 若悬停后深度缓慢漂移：先调 `ff.buoyancy_trim`（正=向下配平，微调），再用 ki 补残差。

### 3.5 阶段 4：档位升迁（mode1 → 2 → 3）

每档在**同工况**下对比，观察"更稳 / 误差更小 / 抗扰更好"：

1. **mode2（+前馈）**：surge/sway 匀速指令下前馈抵消稳态阻力。观察：同 kp 下超调是否减小、匀速误差是否变小；
2. **mode3（+DOB）**：先小带宽开一两个通道（如 `yaw.dob_bandwidth 0.5`），确认无异常再加到 1~2；
   - DOB 效果看：加扰动（如缆绳拉一下、人为压深）后，艇恢复得快不快、稳不稳；
   - DOB 数值依赖 `mass_eff/damp_eff`（⚠ 占位），若振荡加剧 → 降带宽或先关（置 0），不要硬扛；
3. 升档前把艇稳定、摇杆回中；升档后先观察 10 s 再操作。

### 3.6 阶段 5：航行功能验证（可选）

系泊稳定后：小速度前进（vx ≤0.3 m/s）→ 转向 → 深度变化 → 综合机动。每步回中观察保持性能。**全程注意 DVL 有效**。

---

## 4. 实机常见问题速查

| 现象 | 可能原因 | 处理 |
|---|---|---|
| 所有输出恒 0，日志刷"传感器数据无效" | IMU/DVL/深度计 `connection_status≠1`（D5 全停） | 查传感器硬件/话题；DVL 是否底跟踪有效 |
| 恒 0，日志刷"推进器故障" | 主/辅推 fault_status≠0 | 查 CAN/电调 |
| 恒 0，日志刷"指令超时" | 遥控通道断了 >1 s | 查地面站链路 |
| 切模式 1 没反应 | 上位机发的模式值≠1；或节点没激活 | `ros2 lifecycle get`；检查 `/hal/modecontrol` echo |
| 键盘模式与 PID 互相抢 | 模式语义混乱 | 确认上位机只发 1/2/3/4 且 1 与 3 互斥（代码已互斥，若上位机乱发会互相踢） |
| 某通道不动 | enable 被关 / 增益为 0 / 该轴分配矩阵全 0 | `ros2 param get enable_xxx`；echo cmd 看对应百分比 |
| 输出持续饱和 ±100% | 需求超物理上限；或 thrust_limits 填小 | 降增益/降任务量；核对 thrust_limits 与 alloc_matrix 量纲 |
| 抖动/啸叫 | kp 过大、DOB 带宽过大、角速率估计噪声 | 降 kp；降/关 DOB；加大 pid_deriv_alpha |
| 稳态漂移 | ki 不足 / 浮力配平不对 / 水流 | depth 先调 buoyancy_trim；再 ki |
| 艏向永远转不到目标 | ch4 映射理解错（绝对航向） | 对准目标再切 PID；或 enable_yaw false 先做他轴 |
| 重启后参数回到旧值 | 在线 set 不持久 | 按 §5 dump→写回 yaml |

---

## 5. 调参收尾：把本次成果固化（重要）

在线 `param set` / `param load` 的值**重启即丢**。结束调试前把终值固化到开机基线：

```bash
# 1) 把当前节点参数存盘（用于存档/对比; 注意 dump 只含与默认值不同的参数）
ros2 param dump /bsp_motioncontrol_node --output-dir ~/auv_tuning
# 生成: ~/auv_tuning/bsp_motioncontrol_node.yaml

# 2) 把"要固化的终值"并入开机基线
#    方案A(包内基线, 待 autostart 负责人落地后): 编辑随包 config/bsp_motioncontrol.yaml 逐项替换,
#                     然后 colcon build --packages-select bsp 重新安装;
#    方案B(外置覆盖): 按 autostart 负责人提供的机制部署覆盖文件
#                     (见 docs/handoff_autostart_change.md; 勿假设 ~/auv_tuning 在主控存在)
```

> 说明：本开发机上的 `~/auv_tuning/*` 只是本地存档，**不会自动出现在主控上**；
> 运行期在线调参（param set/load）不依赖任何文件，主控上直接用。
> 开机基线的正式落地方式以 autostart 负责人方案为准（需求见 `docs/handoff_autostart_change.md`）。
> 也推荐顺手把 dump 文件连同日期存档（如 `~/auv_tuning/2026-09-02_mode3.yaml`），方便回滚对比。
> 每次正式试验建议先 `ros2 param dump` 存一份"调前基线"。

---

## 6. ⚠ 占位参数替换流程（G1–G4）

yaml 中标 ⚠ 的值是 python 版移植的**占位值**，直接影响 DOB 与分配的物理正确性，建议尽早替换：

| 组 | 参数 | 现在值(⚠) | 真实来源 |
|---|---|---|---|
| G2 | `thrust_limits` [主推,5×辅推] N | 441, 69×5 | 台架实测各推正/反最大推力 |
| G3 | `surge.mass_eff` | 275 kg | 质量+附加质量(或实测) |
| G3 | `sway/depth.mass_eff` | 526.8 kg | 同上 |
| G3 | `yaw.mass_eff` / `pitch.mass_eff` | 472.9 / 472.0 kg·m² | 转动惯量(模型或摆测) |
| G3 | `roll.mass_eff` / `damp_eff` | **4.345 / 0.434 ⚠⚠ 疑似笔误** | 优先核对: 与 pitch/yaw 同量级(几十~几百 kg·m²)才合理 |
| G4 | `*.damp_eff` | 47~125 | 阻尼辨识/CFD |
| G1 | `alloc_matrix` 36 元素 | 占位几何 | 推进器安装位置/方向计算(§设计文档6.1) |
| G2/G7 | 输出符号与正反转 | — | 台架单推验证(§3.1 第3条) |

替换步骤：拿到实测/模型值 → 更新基线参数（当前先改 `docs/bsp_motioncontrol_reference.yaml` 留档；
随包基线落地后改 `config/bsp_motioncontrol.yaml` 并 `colcon build --packages-select bsp`）→ 重启节点验证。

> roll 特别注意：若 roll 惯量真的是 ~4 kg·m² 说明数值单位/来源有误（一艘 AUV 的横摇惯量通常与纵摇同量级）。**在 mode3 之前必须先核对**，否则 roll 通道 DOB 会以错误模型估计扰动。

---

## 7. 调参记录模板（建议打印/抄录）

```
日期/水域/海况: ____________   艇号: ______   版本/基线yaml: ______
控制器模式: mode[1/2/3]   启用的通道: [yaw depth surge sway roll pitch]
┌──────┬───────────┬──────────────┬──────────────────┬──────────┐
│ 通道 │ 参数(旧→新)│ 现象/数据     │ 结论(留/回退)     │ 备注     │
├──────┼───────────┼──────────────┼──────────────────┼──────────┤
│ yaw  │ kp 400→300│ 超调明显减小  │ 留               │          │
└──────┴───────────┴──────────────┴──────────────────┴──────────┘
收尾: 已 dump 至 ____________ ; 已写回 yaml [是/否] ; 下次开机验证 [ ]
```

---

## 8. 最小命令速查

```bash
# 节点/状态
ros2 lifecycle get /bsp_motioncontrol_node
ros2 node list

# 传感器
ros2 topic hz /hal/inertialnavi /hal/dvl /hal/depthsensor
ros2 topic echo /hal/inertialnavi --once

# 输出
ros2 topic echo /hal/thruster/cmd

# 调参
ros2 param list /bsp_motioncontrol_node
ros2 param get  /bsp_motioncontrol_node <参数>
ros2 param set  /bsp_motioncontrol_node <参数> <值>      # double 带小数点
ros2 param dump /bsp_motioncontrol_node --output-dir ~/auv_tuning

# 控制档位/通道
ros2 param set /bsp_motioncontrol_node controller_mode 3     # 开DOB(先置带宽)
ros2 param set /bsp_motioncontrol_node yaw.dob_bandwidth 0.0 # 关yaw的DOB
ros2 param set /bsp_motioncontrol_node enable_depth false    # 冻结depth通道

# 重建参数基线(改完 yaml 后)
cd ~/UVMS_WS && source /opt/ros/humble/setup.bash && colcon build --packages-select bsp
```
