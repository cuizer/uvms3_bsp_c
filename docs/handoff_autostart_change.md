# 需求交接（v2）：bsp_motioncontrol_node 启动参数加载方式（给自启动/launch 负责人）

> 背景要点：
> - 开发/调参在开发机进行；**autostart 实际运行在航行器主控上**，主控与开发机不是同一台，
>   开发机上存在的任何文件（如 `~/auv_tuning/*`）默认不会出现在主控上；
> - `uvms_autostart.launch.py` 由贵方维护，我方**不保留对该文件的任何本地改动**（已回退），
>   以下为需求说明，实现方式与落地由贵方决定；
> - bsp_motioncontrol_node（运动控制节点）已具备**运行期在线调参**能力
>   （`ros2 param set` / `ros2 param load`，无需重编译、无需重启节点）——此能力不依赖任何文件。

---

## 一、需求背景

运动控制节点运行期可在线调参，但**调好的参数在下次开机后是否保留**，取决于节点启动时
加载哪些参数。目前节点参数来自代码内默认值；希望改为**由一份随包发布的默认基线 YAML
统一提供**，并**可选**支持"外置可编辑基线覆盖"，使调参成果可以不改代码就固化到开机参数。

## 二、需求内容（功能要求）

1. **默认参数基线随包发布**：在 bsp 包内提供 `config/bsp_motioncontrol.yaml`
   （全量参数基线，与节点 declare 参数一一对应，double 带小数点，顶层键 `bsp_motioncontrol_node`；
   参考文件见 `docs/bsp_motioncontrol_reference.yaml`），并在 CMakeLists 中安装
   （`install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)`）。
2. **autostart 启动 bsp_motioncontrol_node 时加载该基线**：
   `bsp_motion = LifecycleNode(..., parameters=[<配置文件路径>])`。
   不要求在 launch 内联 PID/DOB 等调参参数（避免双重来源）。
3. **可选的外置覆盖机制（如贵方需要"免重编译改开机基线"）**：
   实现方式不限，可由贵方按主控的部署/配置管理惯例决定（例如固定绝对路径、环境变量、
   现有配置中心下发），**但必须满足**：
   - 外置文件**缺失时静默回退到包内默认基线**，绝不允许因外置文件缺失导致启动失败；
   - 路径**不依赖登录用户的家目录 `~`**（autostart 的启动用户可能与开发用户不同，
     开发机上存在的路径在主控上不成立）。
   - 若暂不做外置覆盖：仅实现第 1、2 条即可，属可接受的最小版本。
4. **不改动自启动其他行为**：仅 bsp_motioncontrol_node 的参数来源变化，
   生命周期分组、启动顺序、其余节点（bsp_comm / bsp_remotecontrol / startup_manager / hal 各驱动）一律不动。
5. 配置文件损坏/半截时的行为由贵方按惯例定（建议：报清晰错误拒绝启动该节点，或回退默认并告警）。

## 三、验收标准（以主控为准）

1. 主控上不存在任何外置覆盖文件时：autostart 正常启动，
   `ros2 param get /bsp_motioncontrol_node pid_yaw.kp` 返回包内基线值（400.0）；
2. （若实现外置覆盖）在主控部署位置放置修改过的基线（如 kp 改为 300.0）后重启 launch：
   节点加载 300.0，且全程无需重新编译 bsp；
3. 运行期在线调参（param set/load）在 autostart 启动的节点上同样可用；
4. 除运动节点参数来源外，autostart 行为与改动前一致。

## 四、最小实现参考（方案 A：仅包内基线，无外置覆盖）

```python
# uvms_autostart.launch.py 中：
bsp_motion = LifecycleNode(
    package="bsp",
    executable="bsp_motioncontrol_node",
    name="bsp_motioncontrol_node",
    output="screen",
    parameters=[
        os.path.join(
            get_package_share_directory("bsp"),
            "config", "bsp_motioncontrol.yaml"),
    ],
)
```

## 五、可选实现参考（方案 B：包内基线 + 外置覆盖，主控路径不依赖 ~）

```python
# 启动前由部署脚本/环境变量指定外置基线；未指定或文件不存在 → 用包内默认。
import os
_bsp_installed_cfg = os.path.join(
    get_package_share_directory("bsp"), "config", "bsp_motioncontrol.yaml")
_bsp_override_cfg = os.environ.get("BSP_MOTION_CFG", "")   # 例: /opt/auv/cfg/bsp_motioncontrol.yaml
bsp_motion_cfg = (
    _bsp_override_cfg if _bsp_override_cfg and os.path.exists(_bsp_override_cfg)
    else _bsp_installed_cfg)

bsp_motion = LifecycleNode(
    package="bsp", executable="bsp_motioncontrol_node",
    name="bsp_motioncontrol_node", output="screen",
    parameters=[bsp_motion_cfg],
)
```

> 说明：`docs/motion_control_debug_guide.md` 中出现的 `~/auv_tuning/...` 仅为**开发机本地**
> 调参收尾的示例路径，不代表主控约定；主控上的固化路径以贵方方案为准。

## 六、配套现状

- 我方本地已回退对 `uvms_autostart.launch.py`、`CMakeLists.txt` 的全部改动，仓库中不再包含
  autostart 相关修改，贵方可直接基于本需求实现；
- 参考基线文件留存于 `docs/bsp_motioncontrol_reference.yaml`（内容=节点当前全部默认参数）。
