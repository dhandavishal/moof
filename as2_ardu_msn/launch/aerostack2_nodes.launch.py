#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
                'fcu_url': 'udp://:14555@127.0.0.1:14550',  # Bind to port 14555, send to SITL port 14550
                'tgt_system': '1',
                'tgt_component': '1',
                'namespace': 'mavros',  # Use a specific namespace for MAVROS
                # Disable automatic mission download to prevent stale mission issues
                'enable_partial_push': 'false',
                'enable_partial_download': 'false',
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
                    'max_thrust': 15.0,
                    'min_thrust': 0.0,
                    'platform': 'mavlink',
                    'base_frame': 'base_link',
                    'global_frame': 'earth',
                    'odom_frame': 'odom',
                    'mavros_namespace': 'mavros',  # Point to MAVROS namespace (without leading slash)
                    'control_modes_file': os.path.join(pkg_dir, 'config', 'control_modes.yaml'),
                    'external_odom': True,
                    'use_sim_time': False,
                }
            ],
            output='screen'
        ),
        
        # State estimator with fixed frame names
        Node(
            package='as2_state_estimator',
            executable='as2_state_estimator_node',
            name='state_estimator',
            namespace=drone_namespace,
            parameters=[{
                'plugin_name': 'raw_odometry',
                'base_frame': 'drone0/base_link',  # Full frame name with namespace
                'global_frame': 'earth',
                'odom_frame': 'drone0/odom',  # Full frame name with namespace
                'enable_tf': True,
                'use_sim_time': False,
            }],
            output='screen'
        ),
        
        # Motion controller with correct plugin name
        Node(
            package='as2_motion_controller',
            executable='as2_motion_controller_node',
            name='motion_controller',
            namespace=drone_namespace,
            parameters=[{
                'controller_plugin_name': 'pid_speed_controller::Plugin',  # Fixed plugin name
                'cmd_freq': 30.0,
                'info_freq': 10.0,
                'motion_reference_adder_plugin_name': 'position_motion',
                'use_sim_time': False,
                # Add PID parameters
                'position_control': {
                    'p': [1.0, 1.0, 1.0],
                    'i': [0.0, 0.0, 0.0],
                    'd': [0.5, 0.5, 0.5],
                },
                'velocity_control': {
                    'p': [1.0, 1.0, 1.0],
                    'i': [0.0, 0.0, 0.0],
                    'd': [0.0, 0.0, 0.0],
                },
                'yaw_control': {
                    'p': 1.0,
                    'i': 0.0,
                    'd': 0.0,
                },
            }],
            output='screen'
        ),
    ])