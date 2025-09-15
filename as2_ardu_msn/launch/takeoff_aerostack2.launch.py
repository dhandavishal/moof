#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directory
    pkg_name = 'as2_ardu_msn'
    pkg_dir = get_package_share_directory(pkg_name)
    mavros_dir = get_package_share_directory('mavros')
    
    # Declare launch arguments
    drone_namespace_arg = DeclareLaunchArgument('drone_namespace', default_value='drone0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    takeoff_config_arg = DeclareLaunchArgument(
        'takeoff_config',
        default_value=os.path.join(pkg_dir, 'config', 'takeoff_config.yaml'),
        description='Path to the takeoff configuration file.'
    )

    # Launch Configurations
    drone_namespace = LaunchConfiguration('drone_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    takeoff_config = LaunchConfiguration('takeoff_config')

    # MAVROS Launch
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_dir, 'launch', 'apm.launch')
        ),
        launch_arguments={
            'fcu_url': 'udp://0.0.0.0:14550@',  # Just listen, don't specify target
            'tgt_system': '1',
            'tgt_component': '1',
        }.items()
    )

    # Aerostack2 Platform Node
    platform_node = Node(
        package='as2_platform_mavlink',
        executable='as2_platform_mavlink_node',
        name='platform',
        namespace=drone_namespace,
        parameters=[{
            'max_thrust': 15.0,
            'min_thrust': 0.0,
            'platform': 'mavlink',
            'base_frame': 'base_link',
            'global_frame': 'earth',
            'odom_frame': 'odom',
            'mavros_namespace': '/mavros',
            'control_modes_file': os.path.join(pkg_dir, 'config', 'control_modes.yaml'),
            'external_odom': True,
            'use_sim_time': False,
        }],
        output='screen'
    )

    # State Estimator
    state_estimator_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('as2_state_estimator'), 'launch', 'state_estimator_launch.py')
        ),
        launch_arguments={
            'namespace': drone_namespace,
            'plugin_name': 'raw_odometry',
            'use_sim_time': use_sim_time,
            'config_file': os.path.join(pkg_dir, 'config', 'state_estimator.yaml'),
        }.items()
    )

    # Motion Controller
    motion_controller_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('as2_motion_controller'), 'launch', 'controller_launch.py')
        ),
        launch_arguments={
            'namespace': drone_namespace,
            'plugin_name': 'pid_speed_controller',
            'use_sim_time': use_sim_time,
            'plugin_config_file': os.path.join(pkg_dir, 'config', 'pid_speed_controller.yaml'),
        }.items()
    )

    # Motion Behaviors
    motion_behaviors_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('as2_behaviors_motion'), 'launch', 'motion_behaviors_launch.py')
        ),
        launch_arguments={
            'namespace': drone_namespace,
            'use_sim_time': use_sim_time,
            'config_file': os.path.join(pkg_dir, 'config', 'motion_behaviors.yaml'),
        }.items()
    )

    # Takeoff Mission Node
    takeoff_mission_node = Node(
        package=pkg_name,
        executable='takeoff_mission.py',
        name='takeoff_mission_node',
        namespace=drone_namespace,
        output='screen',
        arguments=['--config', takeoff_config],
    )

    return LaunchDescription([
        drone_namespace_arg,
        use_sim_time_arg,
        takeoff_config_arg,
        
        mavros_launch,
        platform_node,
        state_estimator_launch,
        motion_controller_launch,
        motion_behaviors_launch,
        takeoff_mission_node,
    ])
