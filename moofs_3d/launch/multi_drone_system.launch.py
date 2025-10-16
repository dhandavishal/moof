#!/usr/bin/env python3
"""
Complete Multi-Drone System Launch
Launches MAVROS for multiple drones and monitoring node
Note: SITL instances must be started separately
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_system_nodes(context, *args, **kwargs):
    """Generate all system nodes"""
    
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    base_fcu_port = 14550
    base_gcs_port = 14555
    
    nodes = []
    
    # Generate MAVROS nodes
    for i in range(num_drones):
        drone_id = i
        namespace = f'drone_{drone_id}'
        
        fcu_url = f'udp://:{base_fcu_port + (drone_id * 10)}@localhost:{base_gcs_port + (drone_id * 10)}'
        
        mavros_params = {
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': drone_id + 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
            'system_id': 255,
            'component_id': 240,
            'startup_px4_usb_quirk': False,
        }
        
        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=namespace,
            output='screen',
            parameters=[mavros_params],
        )
        
        nodes.append(mavros_node)
    
    # Add monitoring node
    monitor_node = Node(
        package='moofs_3d',
        executable='multi_drone_monitor',
        name='multi_drone_monitor',
        output='screen',
        parameters=[{'num_drones': num_drones}]
    )
    nodes.append(monitor_node)
    
    return nodes


def generate_launch_description():
    """Generate complete launch description"""
    
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones in the system'
    )
    
    ld = LaunchDescription()
    ld.add_action(num_drones_arg)
    ld.add_action(OpaqueFunction(function=generate_system_nodes))
    
    return ld
