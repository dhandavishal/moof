#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # It's good practice to get the package directory to locate config files
    # Replace 'your_package_name' with the actual name of your ROS 2 package
    pkg_name = 'as2_ardu_msn' # <--- CHANGE THIS TO YOUR PACKAGE NAME
    pkg_dir = get_package_share_directory(pkg_name)
    
    mavros_dir = get_package_share_directory('mavros')
    
    # Declare launch arguments
    drone_namespace_arg = DeclareLaunchArgument('drone_namespace', default_value='drone0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=os.path.join(pkg_dir, 'config', 'mission_config.yaml'),
        description='Path to the mission configuration file.'
    )

    # Launch Configurations
    drone_namespace = LaunchConfiguration('drone_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_config = LaunchConfiguration('mission_config')

    # MAVROS Launch (fixed for SITL connection)
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(mavros_dir, 'launch', 'apm.launch')
        ),
        launch_arguments={
            'fcu_url': 'udp://:14555@127.0.0.1:14550',
            'tgt_system': '1',
            'tgt_component': '1',
        }.items()
    )

    # Aerostack2 Platform Node (delayed to allow MAVROS to fully initialize)
    platform_node = TimerAction(
        period=5.0,  # 5 second delay
        actions=[
            Node(
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
        ]
    )

    # Aerostack2 Core Stack (Estimator, Controller, Behaviors)
    # This assumes standard config files are present in your package's 'config' directory
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

    # Your Custom Mission Node
    survey_mission_node = Node(
        package=pkg_name,
        executable='basic_survey_mission.py', # Assumes this is an entry point in setup.py
        name='survey_mission_node',
        namespace=drone_namespace,
        output='screen',
        arguments=['--config', mission_config],
    )
    return LaunchDescription([
        drone_namespace_arg,
        use_sim_time_arg,
        mission_config_arg,
        
        mavros_launch,
        platform_node,
        state_estimator_launch,
        motion_controller_launch,
        motion_behaviors_launch,
        survey_mission_node,
    ])