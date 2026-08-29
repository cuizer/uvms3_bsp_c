"""Bring up BSP remote-control and PID motion-control nodes."""

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():

    # ==============================================================
    # 开环遥控节点
    # ==============================================================
    remote = LifecycleNode(
        package="bsp",
        executable="bsp_remotecontrol_node",
        name="bsp_remotecontrol_node",
        namespace="",
        output="screen",
        parameters=[
            {
                # Topic
                "mode_topic": "/hal/modecontrol",
                "remote_topic": "/hal/remotecontrol",

                # HalModeControl.msg 只有:
                # uint8 modecontrol_cmd
                #
                # 这里由节点参数规定模式编号:
                # 1 -> 开环遥控
                "mode_keyboard_value": 1,

                # 控制周期
                "control_rate_hz": 20.0,
                "cmd_timeout_s": 0.5,

                # 控制量缩放
                "surge_scale": 300.0,
                "sway_scale": 160.0,
                "heave_scale": 80.0,
                "yaw_scale": 5.0,

                # 平滑参数
                "smoothing.enable": True,
                "smoothing.slew_Fx": 600.0,
                "smoothing.slew_Fy": 320.0,
                "smoothing.slew_Fz": 160.0,
                "smoothing.slew_Mz": 10.0,
                "smoothing.thruster_tau": 0.2,
            }
        ],
    )

    # ==============================================================
    # 闭环 PID 运动控制节点
    # ==============================================================
    controller = LifecycleNode(
        package="bsp",
        executable="bsp_motioncontrol_node",
        name="bsp_motioncontrol_node",
        namespace="",
        output="screen",
        parameters=[
            {
                # Topic
                "mode_topic": "/hal/modecontrol",
                "remote_topic": "/hal/remotecontrol",

                # HalModeControl.msg 只有:
                # uint8 modecontrol_cmd
                #
                # 这里由节点参数规定模式编号:
                # 2 -> PID 闭环控制
                "mode_pid_value": 2,
            }
        ],
    )

    return LaunchDescription([
        remote,
        controller,
    ])
