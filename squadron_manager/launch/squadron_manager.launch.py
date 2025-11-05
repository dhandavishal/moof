#!/usr/bin/env python3
"""
Launch file for Squadron Manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for squadron manager"""
    
    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones in squadron'
    )
    
    allocation_strategy_arg = DeclareLaunchArgument(
        'allocation_strategy',
        default_value='nearest',
        description='Task allocation strategy (greedy, nearest, load_balanced, capability_based)'
    )
    
    enable_formations_arg = DeclareLaunchArgument(
        'enable_formations',
        default_value='true',
        description='Enable formation control'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('squadron_manager'),
            'config',
            'squadron_config.yaml'
        ]),
        description='Path to squadron configuration file'
    )
    
    # Squadron Manager node
    squadron_manager_node = Node(
        package='squadron_manager',
        executable='squadron_manager_node',
        name='squadron_manager',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'num_drones': LaunchConfiguration('num_drones'),
                'allocation_strategy': LaunchConfiguration('allocation_strategy'),
                'enable_formations': LaunchConfiguration('enable_formations'),
            }
        ],
        emulate_tty=True
    )
    
    return LaunchDescription([
        num_drones_arg,
        allocation_strategy_arg,
        enable_formations_arg,
        config_file_arg,
        squadron_manager_node
    ])
