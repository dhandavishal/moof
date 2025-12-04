#!/usr/bin/env python3
"""
Complete Multi-Drone System Launch
Launches MAVROS for multiple drones and monitoring node
Note: SITL instances must be started separately
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_system_nodes(context, *args, **kwargs):
    """Generate all system nodes"""
    
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    fcu_protocol = LaunchConfiguration('fcu_protocol').perform(context)
    fcu_host = LaunchConfiguration('fcu_host').perform(context)
    
    base_fcu_port = 14570
    base_gcs_port = 14575
    base_tcp_port = 5760
    
    # Get MAVROS package directory
    mavros_dir = get_package_share_directory('mavros')
    
    nodes = []
    
    # Generate MAVROS nodes using apm.launch
    for i in range(num_drones):
        drone_id = i
        namespace = f'drone_{drone_id}'
        
        if fcu_protocol == 'tcp':
            port = base_tcp_port + (drone_id * 10)
            fcu_url = f'tcp://{fcu_host}:{port}'
        else:
            fcu_url = f'udp://:{base_fcu_port + (drone_id * 10)}@{fcu_host}:{base_gcs_port + (drone_id * 10)}'
        
        # Use GroupAction with PushRosNamespace and IncludeLaunchDescription
        mavros_group = GroupAction([
            PushRosNamespace(namespace),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(mavros_dir, 'launch', 'apm.launch')
                ),
                launch_arguments={
                    'fcu_url': fcu_url,
                    'tgt_system': str(drone_id + 1),
                    'tgt_component': '1',
                }.items()
            )
        ])
        
        nodes.append(mavros_group)
    
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

    fcu_protocol_arg = DeclareLaunchArgument(
        'fcu_protocol',
        default_value='udp',
        description='Protocol to communicate with FCU (udp/tcp)'
    )

    fcu_host_arg = DeclareLaunchArgument(
        'fcu_host',
        default_value='host.docker.internal',
        description='Host address of the FCU (SITL) - use host.docker.internal for Mac host SITL'
    )
    
    ld = LaunchDescription()
    ld.add_action(num_drones_arg)
    ld.add_action(fcu_protocol_arg)
    ld.add_action(fcu_host_arg)
    ld.add_action(OpaqueFunction(function=generate_system_nodes))
    
    return ld
