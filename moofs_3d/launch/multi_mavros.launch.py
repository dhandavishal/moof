#!/usr/bin/env python3
"""
Multi-MAVROS Launch File
Launches multiple MAVROS instances for multi-drone control
Each drone gets unique namespace and FCU connection ports
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_mavros_nodes(context, *args, **kwargs):
    """Generate MAVROS nodes based on num_drones parameter"""
    
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    base_fcu_port = 14550
    base_gcs_port = 14555
    
    nodes = []
    
    for i in range(num_drones):
        drone_id = i
        namespace = f'drone_{drone_id}'
        
        # Calculate unique ports for this drone
        fcu_url = f'udp://:{base_fcu_port + (drone_id * 10)}@localhost:{base_gcs_port + (drone_id * 10)}'
        
        # MAVROS parameters
        mavros_params = {
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': drone_id + 1,  # Must match SYSID_THISMAV in SITL
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
            'system_id': 255,
            'component_id': 240,
            'startup_px4_usb_quirk': False,
        }
        
        # Create MAVROS node
        mavros_node = Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace=namespace,
            output='screen',
            parameters=[mavros_params],
            remappings=[
                # Remap topics to be within namespace
                ('mavros/state', f'/{namespace}/mavros/state'),
                ('mavros/local_position/pose', f'/{namespace}/mavros/local_position/pose'),
                ('mavros/global_position/global', f'/{namespace}/mavros/global_position/global'),
                ('mavros/battery', f'/{namespace}/mavros/battery'),
            ],
        )
        
        nodes.append(mavros_node)
        
        print(f"Configured MAVROS for {namespace}:")
        print(f"  FCU URL: {fcu_url}")
        print(f"  Target System ID: {drone_id + 1}")
        print()
    
    return nodes


def generate_launch_description():
    """Generate launch description with configurable number of drones"""
    
    # Declare launch arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones to launch MAVROS for'
    )
    
    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='true',
        description='Enable ROS logging'
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(num_drones_arg)
    ld.add_action(enable_logging_arg)
    
    # Add MAVROS nodes (generated dynamically)
    ld.add_action(OpaqueFunction(function=generate_mavros_nodes))
    
    return ld
