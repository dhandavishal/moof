#!/usr/bin/env python3
"""
Test launch for FAL + TEE only (without MAVROS)
Useful for testing TEE and FAL integration without MAVROS dependency
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch for FAL and TEE only."""
    
    # FAL (immediate start)
    fal = Node(
        package='flight_abstraction',
        executable='fal_node',
        name='fal_node',
        namespace='/drone_0',
        output='screen',
        parameters=[{'drone_namespace': '/drone_0'}]
    )
    
    # TEE (after 3s to let FAL initialize)
    tee = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='task_execution',
                executable='tee_node',
                name='tee_node',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        fal,
        tee
    ])
