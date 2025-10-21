#!/usr/bin/env python3
"""Launch file for Flight Abstraction Layer (FAL) node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for FAL node."""
    
    # Declare launch arguments
    drone_namespace_arg = DeclareLaunchArgument(
        'drone_namespace',
        default_value='/drone_0',
        description='Namespace for the drone'
    )
    
    # Create FAL node
    fal_node = Node(
        package='flight_abstraction',
        executable='fal_node',
        name='fal_node',
        output='screen',
        parameters=[],
        arguments=[LaunchConfiguration('drone_namespace')],
    )
    
    return LaunchDescription([
        drone_namespace_arg,
        fal_node,
    ])
