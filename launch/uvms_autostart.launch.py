"""Unified lower-computer autostart bringup.

This launch file:
1. Configures SocketCAN interfaces before ROS 2 hardware nodes start.
2. Starts lifecycle-capable HAL/BSP nodes in the unconfigured state.
3. Starts bsp_startupmanager_node. Battery and BSP communication are activated
   first; remaining lifecycle groups are activated from actual power feedback.
"""

import os
import shutil
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


# ==============================================================================
# CAN initialization
# ==============================================================================

def setup_can_interfaces(context):
    """Configure SocketCAN interfaces before starting ROS 2 nodes."""

    ip_command = shutil.which("ip")

    if ip_command is None:
        raise RuntimeError(
            "[CAN SETUP] Cannot find 'ip' command. "
            "Please install/configure iproute2."
        )

    can_configs = [
        (
            "can2",
            int(LaunchConfiguration("can2_bitrate").perform(context)),
        ),
        (
            "can3",
            int(LaunchConfiguration("can3_bitrate").perform(context)),
        ),
        (
            "can4",
            int(LaunchConfiguration("can4_bitrate").perform(context)),
        ),
    ]

    print("")
    print("============================================================")
    print("[CAN SETUP] Starting SocketCAN initialization")
    print("============================================================")

    for interface, bitrate in can_configs:

        # ----------------------------------------------------------
        # 1. Check whether the CAN interface exists
        # ----------------------------------------------------------
        result = subprocess.run(
            [ip_command, "link", "show", interface],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        if result.returncode != 0:
            raise RuntimeError(
                "[CAN SETUP] Interface '{}' does not exist.".format(interface)
            )

        print(
            "[CAN SETUP] Configuring {} -> {} bit/s".format(
                interface,
                bitrate,
            )
        )

        # ----------------------------------------------------------
        # 2. Bring interface down
        #
        # check=False:
        # Interface may already be DOWN, which is acceptable.
        # ----------------------------------------------------------
        subprocess.run(
            [
                "sudo",
                "-n",
                ip_command,
                "link",
                "set",
                interface,
                "down",
            ],
            check=False,
        )

        # ----------------------------------------------------------
        # 3. Configure CAN bitrate
        # ----------------------------------------------------------
        try:
            subprocess.run(
                [
                    "sudo",
                    "-n",
                    ip_command,
                    "link",
                    "set",
                    interface,
                    "type",
                    "can",
                    "bitrate",
                    str(bitrate),
                ],
                check=True,
            )

            # ------------------------------------------------------
            # 4. Bring interface up
            # ------------------------------------------------------
            subprocess.run(
                [
                    "sudo",
                    "-n",
                    ip_command,
                    "link",
                    "set",
                    interface,
                    "up",
                ],
                check=True,
            )

        except subprocess.CalledProcessError as exception:
            raise RuntimeError(
                "[CAN SETUP] Failed to configure {} at {} bit/s. "
                "Check sudo permission and CAN hardware.".format(
                    interface,
                    bitrate,
                )
            ) from exception

        # ----------------------------------------------------------
        # 5. Print configured interface state
        # ----------------------------------------------------------
        verify = subprocess.run(
            [
                ip_command,
                "-details",
                "link",
                "show",
                interface,
            ],
            capture_output=True,
            text=True,
        )

        if verify.returncode == 0:
            print(verify.stdout.strip())

        print(
            "[CAN SETUP] {} configured successfully.".format(interface)
        )

    print("============================================================")
    print("[CAN SETUP] All SocketCAN interfaces are ready")
    print("============================================================")
    print("")

    return []


# ==============================================================================
# Main launch
# ==============================================================================

def generate_launch_description():

    hal_share = get_package_share_directory("hal")

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------

    autostart = LaunchConfiguration("autostart")
    require_power_feedback = LaunchConfiguration("require_power_feedback")
    power_stabilize_ms = LaunchConfiguration("power_stabilize_ms")

    lifecycle_bringup_max_attempts = LaunchConfiguration(
        "lifecycle_bringup_max_attempts"
    )

    lifecycle_bringup_retry_delay_ms = LaunchConfiguration(
        "lifecycle_bringup_retry_delay_ms"
    )

    configure_to_activate_delay_ms = LaunchConfiguration(
        "configure_to_activate_delay_ms"
    )

    # --------------------------------------------------------------------------
    # HAL configuration files
    # --------------------------------------------------------------------------

    binocamera_params = os.path.join(
        hal_share,
        "config",
        "hal_binocamera_node.yaml",
    )

    cabinmotor_params = os.path.join(
        hal_share,
        "config",
        "hal_cabinmotor.yaml",
    )

    # ==========================================================================
    # HAL lifecycle nodes
    # ==========================================================================

    hal_battery = LifecycleNode(
        package="hal",
        executable="hal_battery_node",
        name="hal_battery_node",
        output="screen",
    )

    hal_inertial = LifecycleNode(
        package="hal",
        executable="hal_inertialnavi_node",
        name="hal_inertialnavi_node",
        output="screen",
    )

    hal_light = LifecycleNode(
        package="hal",
        executable="hal_lightcontrol_node",
        name="hal_light_sw_pwm_node",
        output="screen",
        parameters=[{
            "gpio_path": "/dev/gpio/do0/value",
            "active_low": False,
            "pwm_freq_hz": 50,
        }],
    )

    hal_binocamera = LifecycleNode(
        package="hal",
        executable="hal_binocamera_node",
        name="hal_binocamera_node",
        output="screen",
        parameters=[binocamera_params],
    )

    hal_dvl = LifecycleNode(
        package="hal",
        executable="hal_dvl_node",
        name="hal_dvl_node",
        output="screen",
    )

    hal_depthsensor = LifecycleNode(
        package="hal",
        executable="hal_depthsensor_node",
        name="hal_depthsensor_node",
        output="screen",
    )

    hal_cabinmotor = LifecycleNode(
        package="hal",
        executable="hal_cabinmotor_node",
        name="hal_cabinmotor_node",
        output="screen",
        parameters=[cabinmotor_params],
    )

    # --------------------------------------------------------------------------
    # Antenna
    #
    # Explicitly specify can2 even though the C++ default is currently can2.
    # This avoids future changes to the node default affecting the system.
    # --------------------------------------------------------------------------

    hal_antenna = LifecycleNode(
        package="hal",
        executable="hal_antennacontrol_node",
        name="hal_antenna_lifecycle_node",
        output="screen",
        parameters=[{
            "can_interface": "can2",
        }],
    )

    hal_thruster = LifecycleNode(
        package="hal",
        executable="hal_thruster_node",
        name="hal_thruster_node",
        output="screen",
    )

    # ==========================================================================
    # BSP lifecycle nodes
    # ==========================================================================

    bsp_comm = LifecycleNode(
        package="bsp",
        executable="bsp_comm_node",
        name="bsp_comm_node",
        output="screen",
    )

    bsp_remote = LifecycleNode(
        package="bsp",
        executable="bsp_remotecontrol_node",
        name="bsp_remotecontrol_node",
        output="screen",
    )

    bsp_motion = LifecycleNode(
        package="bsp",
        executable="bsp_motioncontrol_node",
        name="bsp_motioncontrol_node",
        output="screen",
    )

    # ==========================================================================
    # Normal BSP node
    # ==========================================================================

    bsp_monitor = Node(
        package="bsp",
        executable="bsp_monitor_node",
        name="bsp_monitor_node",
        output="screen",
    )

    # ==========================================================================
    # Startup manager
    # ==========================================================================

    startup_manager = Node(
        package="bsp",
        executable="bsp_startupmanager_node",
        name="bsp_startupmanager_node",
        output="screen",
        parameters=[{
            "autostart": ParameterValue(
                autostart,
                value_type=bool,
            ),

            "require_power_feedback": ParameterValue(
                require_power_feedback,
                value_type=bool,
            ),

            "power_stabilize_ms": ParameterValue(
                power_stabilize_ms,
                value_type=int,
            ),

            "lifecycle_bringup_max_attempts": ParameterValue(
                lifecycle_bringup_max_attempts,
                value_type=int,
            ),

            "lifecycle_bringup_retry_delay_ms": ParameterValue(
                lifecycle_bringup_retry_delay_ms,
                value_type=int,
            ),

            "configure_to_activate_delay_ms": ParameterValue(
                configure_to_activate_delay_ms,
                value_type=int,
            ),

            # --------------------------------------------------------------
            # Initial group: always activate these two first
            # --------------------------------------------------------------
            "initial_lifecycle_nodes": [
                "/hal_battery_node",
                "/bsp_comm_node",
            ],

            # --------------------------------------------------------------
            # 48 V group
            # --------------------------------------------------------------
            "rail_48v_lifecycle_nodes": [
                "/bsp_remotecontrol_node",
                "/bsp_motioncontrol_node",
            ],

            # --------------------------------------------------------------
            # 12 V group
            # --------------------------------------------------------------
            "rail_12v_lifecycle_nodes": [
                "/hal_inertialnavi_node",
                "/hal_light_sw_pwm_node",
                "/hal_binocamera_node",
            ],

            # --------------------------------------------------------------
            # 24 V group
            # --------------------------------------------------------------
            "rail_24v_lifecycle_nodes": [
                "/hal_dvl_node",
                "/hal_depthsensor_node",
                "/hal_cabinmotor_node",
                "/hal_antenna_lifecycle_node",
            ],

            # --------------------------------------------------------------
            # Thruster group: requires BOTH 72 V and 12 V feedback
            # --------------------------------------------------------------
            "thruster_lifecycle_nodes": [
                "/hal_thruster_node",
            ],
        }],
    )

    # ==========================================================================
    # Launch description
    # ==========================================================================

    return LaunchDescription([

        # ----------------------------------------------------------------------
        # General launch arguments
        # ----------------------------------------------------------------------

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "require_power_feedback",
            default_value="true",
        ),

        DeclareLaunchArgument(
            "power_stabilize_ms",
            default_value="2000",
        ),

        DeclareLaunchArgument(
            "lifecycle_bringup_max_attempts",
            default_value="3",
        ),

        DeclareLaunchArgument(
            "lifecycle_bringup_retry_delay_ms",
            default_value="1000",
        ),

        DeclareLaunchArgument(
            "configure_to_activate_delay_ms",
            default_value="5000",
        ),

        # ----------------------------------------------------------------------
        # CAN configuration
        #
        # can2 : antenna etc.          125 kbit/s
        # can3 :                      500 kbit/s
        # can4 :                        1 Mbit/s
        # ----------------------------------------------------------------------

        DeclareLaunchArgument(
            "can2_bitrate",
            default_value="125000",
        ),

        DeclareLaunchArgument(
            "can3_bitrate",
            default_value="500000",
        ),

        DeclareLaunchArgument(
            "can4_bitrate",
            default_value="1000000",
        ),

        # ----------------------------------------------------------------------
        # IMPORTANT:
        # CAN initialization executes synchronously here.
        #
        # Only after CAN2/CAN3/CAN4 have been successfully configured
        # will the following ROS 2 nodes begin to start.
        # ----------------------------------------------------------------------

        OpaqueFunction(
            function=setup_can_interfaces,
        ),

        # ----------------------------------------------------------------------
        # HAL nodes
        # ----------------------------------------------------------------------

        hal_battery,
        hal_inertial,
        hal_light,
        hal_binocamera,
        hal_dvl,
        hal_depthsensor,
        hal_cabinmotor,
        hal_antenna,
        hal_thruster,

        # ----------------------------------------------------------------------
        # BSP nodes
        # ----------------------------------------------------------------------

        bsp_comm,
        bsp_remote,
        bsp_motion,
        bsp_monitor,

        # ----------------------------------------------------------------------
        # Startup manager
        # ----------------------------------------------------------------------

        startup_manager,
    ])