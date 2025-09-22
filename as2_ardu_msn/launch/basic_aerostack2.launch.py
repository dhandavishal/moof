#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory to locate config files
    pkg_name = 'as2_ardu_msn'
    pkg_dir = get_package_share_directory(pkg_name)
    
    mavros_dir = get_package_share_directory('mavros')
    
    # Declare launch arguments
    drone_namespace_arg = DeclareLaunchArgument('drone_namespace', default_value='drone0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    mission_config_arg = DeclareLaunchArgument(
        'mission_config',
        default_value=os.path.join(pkg_dir, 'config', 'mission_config.yaml'),
        description='Path to the mission configuration file.'
    )

    # Launch Configurations
    drone_namespace = LaunchConfiguration('drone_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mission_config = LaunchConfiguration('mission_config')

    # MAVROS Launch - wrapped in namespace group
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
                'config_yaml': os.path.join(pkg_dir, 'config', 'mavros_timing_fix.yaml'),
            }.items()
        )
    ])

    # Static transform publisher to connect earth to drone0/odom
    # This is a backup in case ground_truth plugin doesn't create this transform
    static_tf_earth_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='earth_to_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'earth', 'drone0/odom'],
        output='screen'
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
                parameters=[
                    os.path.join(pkg_dir, 'config', 'platform_params.yaml'),
                    {
                        'use_sim_time': use_sim_time,
                        'control_modes_file': os.path.join(pkg_dir, 'config', 'control_modes.yaml'),
                    }
                ],
                output='screen'
            )
        ]
    )

    # Aerostack2 Core Stack (Estimator, Controller, Behaviors)
    # Using raw_odometry plugin which is simpler for ArduPilot
    state_estimator_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('as2_state_estimator'), 'launch', 'state_estimator_launch.py')
        ),
        launch_arguments={
            'namespace': drone_namespace,
            'plugin_name': 'raw_odometry',  # Back to raw_odometry - simpler for ArduPilot
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
        executable='basic_survey_mission.py',
        name='survey_mission_node',
        namespace=drone_namespace,
        output='screen',
        arguments=['--config', mission_config],
    )
    
    return LaunchDescription([
        drone_namespace_arg,
        use_sim_time_arg,
        mission_config_arg,
        
        # Start MAVROS first
        mavros_launch,
        
        # Add static TF publisher for earth -> odom connection
        static_tf_earth_to_odom,
        
        # Then platform node
        platform_node,
        
        # Then the state estimator with ground_truth plugin
        state_estimator_launch,
        
        # Then controller and behaviors
        motion_controller_launch,
        motion_behaviors_launch,
        
    # The mission node previously had a 25s delay which caused it to miss early latched/platform state messages.
    # Start it earlier (5s after other core nodes) so subscriptions are active sooner.
    TimerAction(period=15.0, actions=[survey_mission_node]),
    ])