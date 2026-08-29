"""Compatibility alias for the BSP remote motion-control bringup.

New deployments should launch ``bsp_remote_motion_control.launch.py`` directly.
This filename remains valid for existing scripts and includes the same bringup;
it must not be launched alongside the canonical entry point.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    canonical_launch = Path(__file__).with_name("bsp_remote_motion_control.launch.py")
    if not canonical_launch.is_file():
        raise RuntimeError(
            "bsp_remote_motion_control.launch.py is missing from the installed bsp package"
        )

    return LaunchDescription([
        LogInfo(msg=(
            "motion_control.launch.py is a compatibility alias; "
            "including bsp_remote_motion_control.launch.py"
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(canonical_launch))
        ),
    ])
