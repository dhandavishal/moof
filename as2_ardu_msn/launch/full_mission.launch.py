#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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
        
        # Include MAVROS APM launch
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': 'udp://127.0.0.1:14555@',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items()
        ),
        
        # AS2 Platform
        Node(
            package='as2_platform_mavlink',
            executable='as2_platform_mavlink_node',
            name='platform',
            namespace=drone_namespace,
            parameters=[
                {
                    'control_modes_file': os.path.join(pkg_dir, 'config', 'control_modes.yaml'),
                    'mavros_namespace': '',  # MAVROS at root level
                }
            ],
            output='screen'
        ),
        
        # State estimator
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=drone_namespace,
            parameters=[{
                'plugin_name': 'raw_odometry',
                'base_frame': 'base_link',
                'global_frame': 'earth',
                'odom_frame': 'odom'
            }],
            output='screen'
        ),
        
        # Motion controller
        Node(
            package='as2_motion_controller',
            executable='as2_motion_controller_node',
            name='motion_controller',
            namespace=drone_namespace,
            parameters=[{
                'controller_plugin_name': 'pid_speed_controller',
                'cmd_freq': 30.0,
                'info_freq': 10.0,
                'motion_reference_adder_plugin_name': 'position_motion',
            }],
            output='screen'
        ),
    ])