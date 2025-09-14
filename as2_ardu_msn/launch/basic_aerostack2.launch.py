#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('as2_ardu_msn')
    mavros_dir = get_package_share_directory('mavros')
    
    drone_namespace = LaunchConfiguration('drone_namespace', default='drone0')
    
    return LaunchDescription([
        DeclareLaunchArgument('drone_namespace', default_value='drone0'),
        
        # MAVROS APM launch
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': 'udp://127.0.0.1:14550@127.0.0.1:14555',
                'tgt_system': '1',
                'tgt_component': '1',
                'namespace': 'mavros',
            }.items()
        ),
        
        # AS2 Platform
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
        ),
        
        # State estimator
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('as2_state_estimator'), 'launch', 'state_estimator_launch.py')
            ),
            launch_arguments={
                'namespace': drone_namespace,
                'plugin_name': 'raw_odometry',
                'use_sim_time': 'false',
                'config_file': os.path.join(pkg_dir, 'config', 'config.yaml'),
            }.items()
        ),
        
        # Motion controller
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('as2_motion_controller'), 'launch', 'controller_launch.py')
            ),
            launch_arguments={
                'namespace': drone_namespace,
                'plugin_name': 'pid_speed_controller',
                'plugin_config_file': os.path.join(pkg_dir, 'config', 'pid_speed_controller.yaml'),
                'config_file': os.path.join(pkg_dir, 'config', 'config.yaml'),
                'use_sim_time': 'false',
            }.items()
        ),
        
        # Motion behaviors (all in one launch)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(get_package_share_directory('as2_behaviors_motion'), 'launch', 'motion_behaviors_launch.py')
            ),
            launch_arguments={
                'namespace': drone_namespace,
                'config_file': os.path.join(pkg_dir, 'config', 'config.yaml'),
                'use_sim_time': 'false',
            }.items()
        ),
    ])
