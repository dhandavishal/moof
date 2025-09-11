from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Project directory
    pkg_dir = get_package_share_directory("as2_ardu_msn")

    # -------- CLI-selectable namespace --------
    drone_ns = LaunchConfiguration("drone_namespace", default="drone0")

    # -------- MAVROS bridge --------
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        parameters=[
            os.path.join(pkg_dir, "config", "mavros_config.yaml"),
            {  # runtime overrides
                "fcu_url": "udp://127.0.0.1:14555@",
                "tgt_system": 1,
                "tgt_component": 1,
            },
        ],
        output="screen",
    )

    # -------- Aerostack-2 MAVLink platform --------
    platform_node = Node(
        package="as2_platform_mavlink",
        executable="as2_platform_mavlink_node",
        name="platform",
        namespace=drone_ns,
        parameters=[
            {
                "control_modes_file": os.path.join(
                    pkg_dir, "config", "control_modes.yaml"
                ),
                "external_odom": True,  # required by the node
                "mavros_namespace": "",  # MAVROS at root level
                "max_thrust": 15.0,
                "min_thrust": 0.0,
                "platform": "mavlink",
                "base_frame": "base_link",
                "global_frame": "earth",
                "odom_frame": "odom",
            }
        ],
        output="screen",
    )

    # -------- State estimator --------
    state_estimator = Node(
        package="as2_state_estimator",
        executable="as2_state_estimator_node",
        name="state_estimator",
        namespace=drone_ns,
        parameters=[
            {
                "plugin_name": "raw_odometry",
                "enable_tf": True,
                "base_frame": "base_link",
                "global_frame": "earth", 
                "odom_frame": "odom",
            }
        ],
        output="screen",
    )

    # -------- Motion controller --------
    motion_controller = Node(
        package="as2_motion_controller",
        executable="as2_motion_controller_node",
        name="motion_controller",
        namespace=drone_ns,
        parameters=[
            {
                "controller_plugin_name": "pid_speed_controller",
                "cmd_freq": 30.0,
                "info_freq": 10.0,
                "motion_reference_adder_plugin_name": "position_motion",
                "position_control": {
                    "p": [1.0, 1.0, 1.0],
                    "i": [0.0, 0.0, 0.0],
                    "d": [0.5, 0.5, 0.5],
                },
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_namespace", default_value="drone0"),
            mavros_node,
            platform_node,
            state_estimator,
            motion_controller,
        ]
    )
