#!/usr/bin/env python3
"""
Multi-drone system launch file with performance monitoring.

Launches up to 10 drones with:
- MAVROS per drone
- FAL per drone
- TEE per drone
- Performance monitor (centralized)

Usage:
    ros2 launch task_execution multi_drone_system.launch.py num_drones:=3
    ros2 launch task_execution multi_drone_system.launch.py num_drones:=10 log_level:=warn
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_drone_nodes(context, *args, **kwargs):
    """Generate nodes for all drones based on num_drones parameter."""
    
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    
    mavros_dir = get_package_share_directory('mavros')
    nodes = []
    
    for drone_id in range(num_drones):
        drone_namespace = f'/drone_{drone_id}'
        # MAVROS listens on UDP port for SITL to send data
        # Each drone sends to a different port: 14540, 14541, 14542, etc.
        fcu_url = f'udp://:{14540 + drone_id}@'
        tgt_system = str(drone_id + 1)
        
        # Stagger startup to avoid overwhelming the system
        # 8 seconds between each drone for stable initialization
        base_delay = drone_id * 8.0
        
        # TF Transform for this drone (immediate)
        tf_node = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_odom_drone_{drone_id}',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'drone_{drone_id}/odom'],
            output='screen'
        )
        nodes.append(tf_node)
        
        # MAVROS for this drone (with delay)
        mavros_group = TimerAction(
            period=base_delay,
            actions=[
                GroupAction([
                    PushRosNamespace(drone_namespace),
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            os.path.join(mavros_dir, 'launch', 'apm.launch')
                        ),
                        launch_arguments={
                            'fcu_url': fcu_url,
                            'tgt_system': tgt_system,
                            'tgt_component': '1',
                        }.items()
                    )
                ])
            ]
        )
        nodes.append(mavros_group)
        
        # FAL Node (delay after MAVROS)
        fal_node = TimerAction(
            period=base_delay + 3.0,
            actions=[
                Node(
                    package='flight_abstraction',
                    executable='fal_node',
                    name='fal_node',
                    namespace=drone_namespace,
                    output='screen',
                    parameters=[{
                        'drone_namespace': drone_namespace,
                        'update_rate': 20.0,
                    }],
                    arguments=['--ros-args', '--log-level', log_level]
                )
            ]
        )
        nodes.append(fal_node)
        
        # TEE Node (delay after FAL)
        tee_node = TimerAction(
            period=base_delay + 6.0,
            actions=[
                Node(
                    package='task_execution',
                    executable='tee_node',
                    name='task_execution_engine',
                    namespace=drone_namespace,
                    output='screen',
                    parameters=[{
                        'drone_id': drone_id,
                    }],
                    arguments=['--ros-args', '--log-level', log_level]
                )
            ]
        )
        nodes.append(tee_node)
    
    return nodes


def generate_launch_description():
    """Generate launch description for multi-drone system."""
    
    # Declare arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones to launch (1-10)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug/info/warn/error)'
    )
    
    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Enable performance monitoring'
    )
    
    # Performance Monitor Node (centralized) - starts after 10 seconds
    performance_monitor = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='task_execution',
                executable='performance_monitor',
                name='performance_monitor',
                namespace='/monitoring',
                output='screen',
                parameters=[{
                    'sample_rate': 1.0,  # Hz
                    'log_to_file': True,
                    'log_directory': '/tmp/moofs_metrics',
                    'max_drones': 10,
                }],
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        num_drones_arg,
        log_level_arg,
        enable_monitor_arg,
        
        # Dynamic drone nodes
        OpaqueFunction(function=generate_drone_nodes),
        
        # Performance monitor
        performance_monitor,
    ])
