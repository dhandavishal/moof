#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # It's good practice to get the package directory to locate config files
    pkg_name = 'as2_ardu_msn'
    pkg_dir = get_package_share_directory(pkg_name)
    
    mavros_dir = get_package_share_directory('mavros')
    
    # Declare launch arguments
    drone_namespace_arg = DeclareLaunchArgument('drone_namespace', default_value='drone0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false')
    enable_tf_diagnostics_arg = DeclareLaunchArgument('enable_tf_diagnostics', default_value='false')
    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=os.path.join(pkg_dir, 'config', 'mission_config.yaml'),
        description='Path to the mission configuration file.'
    )

    # Launch Configurations
    drone_namespace = LaunchConfiguration('drone_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_tf_diagnostics = LaunchConfiguration('enable_tf_diagnostics')
    mission_config = LaunchConfiguration('mission_config')

    # MAVROS Launch - wrapped in namespace group with improved timesync config
    mavros_launch = GroupAction([
        PushRosNamespace(drone_namespace),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': 'udp://:14555@127.0.0.1:14550',
                'tgt_system': '1',
                'tgt_component': '1',
                'config_yaml': os.path.join(pkg_dir, 'config', 'mavros_timesync.yaml'),
            }.items()
        )
    ])

    # Aerostack2 Platform Node (delayed to allow MAVROS to fully initialize)
    platform_node = TimerAction(
        period=10.0,  # 10 second delay
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
                    'mavros_namespace': 'mavros',  # MAVROS is now in the same namespace
                    'control_modes_file': os.path.join(pkg_dir, 'config', 'control_modes.yaml'),
                    'external_odom': True,
                    'use_sim_time': False,
                    'tf_timeout_threshold': 0.15,  # Increased timeout for TF lookups
                    'publish_rate': 50.0,  # Hz - consistent with state estimator
                }],
                output='screen'
            )
        ]
    )

    # Aerostack2 Core Stack (Estimator, Controller, Behaviors)
    state_estimator_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('as2_state_estimator'), 'launch', 'state_estimator_launch.py')
        ),
        launch_arguments={
            'namespace': drone_namespace,
            'plugin_name': 'ground_truth',  # Using ground_truth plugin from config
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

    # TF Diagnostics Node (optional)
    tf_diagnostics_node = Node(
        package='as2_ardu_msn',
        executable='tf_diagnostics.py',
        name='tf_diagnostics',
        output='screen',
        condition=IfCondition(enable_tf_diagnostics)
    )

    # Your Custom Mission Node
    survey_mission_node = Node(
        package=pkg_name,
        executable='basic_survey_mission.py',
        name='survey_mission_node',
        namespace=drone_namespace,
        output='screen',
        arguments=['--config', mission_config],
    )
    
    return LaunchDescription([
        drone_namespace_arg,
        use_sim_time_arg,
        enable_tf_diagnostics_arg,
        mission_config_arg,
        
        mavros_launch,
        platform_node,
        state_estimator_launch,
        motion_controller_launch,
        motion_behaviors_launch,
        tf_diagnostics_node,
        # The mission node must start last
        TimerAction(period=25.0, actions=[survey_mission_node]),
    ])
