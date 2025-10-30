#!/usr/bin/env python3
"""
Quick test launch for MAVROS + FAL + TEE
Simplified version for testing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate simple test launch."""
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    log_level = LaunchConfiguration('log_level')
    
    # Get MAVROS package directory
    mavros_dir = get_package_share_directory('mavros')
    
    # MAVROS for drone_0 - use apm.launch like the working single_drone_test
    mavros = GroupAction([
        PushRosNamespace('drone_0'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(mavros_dir, 'launch', 'apm.launch')
            ),
            launch_arguments={
                'fcu_url': 'udp://:14550@127.0.0.1:14555',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items()
        )
    ])
    
    # TF Static Transform: map -> odom (critical for MAVROS navigation)
    # This tells ROS where the 'map' frame is relative to MAVROS frames
    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'drone_0/odom'],
        output='screen'
    )
    
    # FAL (after 3s)
    fal = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='flight_abstraction',
                executable='fal_node',
                name='fal_node',
                namespace='/drone_0',
                output='screen',
                parameters=[{'drone_namespace': '/drone_0'}]
            )
        ]
    )
    
    # TEE (after 6s)
    tee = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='task_execution',
                executable='tee_node',
                name='task_execution_engine',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        log_level_arg,
        tf_map_to_odom,  # Launch TF first
        mavros,
        fal,
        tee,
    ])
