#!/usr/bin/env python3
"""
Complete single-drone system launch file.

Launches:
- MAVROS (connects to ArduPilot SITL via TCP)
- TF transform (map -> drone_N/odom)
- FAL (Flight Abstraction Layer)
- TEE (Task Execution Engine)

Usage:
    # Docker environment (connects to sitl_drone0 container)
    ros2 launch task_execution complete_system.launch.py

    # With custom drone ID
    ros2 launch task_execution complete_system.launch.py drone_id:=1

    # Linux native (UDP connection to local SITL)
    ros2 launch task_execution complete_system.launch.py use_docker:=false

Prerequisites:
- Docker: SITL container must be running (./scripts/start_multi_drone.sh 1)
- Native: ArduPilot SITL must be running on UDP port 14550/14555
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_drone_nodes(context, *args, **kwargs):
    """Generate nodes for a single drone based on parameters."""
    
    drone_id = int(LaunchConfiguration('drone_id').perform(context))
    log_level = LaunchConfiguration('log_level').perform(context)
    use_docker = LaunchConfiguration('use_docker').perform(context).lower() == 'true'
    fcu_host = LaunchConfiguration('fcu_host').perform(context)
    connection_protocol = LaunchConfiguration('connection_protocol').perform(context).lower()
    sitl_base_port = int(LaunchConfiguration('sitl_port').perform(context))
    
    # Namespace for this drone
    drone_namespace = f'/drone_{drone_id}'
    
    # FCU URL depends on environment
    if use_docker:
        if connection_protocol == 'udp':
            # Docker: MAVROS connects via UDP to Host SITL
            # Port = base + (drone_id * 10)
            udp_port = sitl_base_port + (drone_id * 10)
            # Bind to local port, target host port
            fcu_url = f'udp://:{udp_port}@{fcu_host}:{udp_port}'
        else:
            # Docker: MAVROS connects TO SITL via TCP
            # Each SITL instance listens on port 5760 + (instance * 10)
            sitl_port = 5760 + (drone_id * 10)
            fcu_url = f'tcp://{fcu_host}:{sitl_port}'
    else:
        # Native Linux: MAVROS connects via UDP to local SITL
        # Base port 14550, increment by 10 for each drone
        udp_port = 14550 + (drone_id * 10)
        fcu_url = f'udp://:{udp_port}@127.0.0.1:{udp_port + 5}'
    
    # Target system ID (1-based)
    tgt_system = str(drone_id + 1)
    
    # Get MAVROS package directory
    mavros_dir = get_package_share_directory('mavros')
    
    nodes = []
    
    # ========================================================================
    # TF Transform: map -> drone_N/odom (Start immediately)
    # ========================================================================
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'map_to_odom_drone_{drone_id}',
        arguments=['0', '0', '0', '0', '0', '0', 'map', f'drone_{drone_id}/odom'],
        output='screen'
    )
    nodes.append(tf_node)
    
    # ========================================================================
    # MAVROS Node (Start immediately, uses apm.launch for ArduPilot)
    # ========================================================================
    mavros_group = GroupAction([
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
    nodes.append(mavros_group)
    
    # ========================================================================
    # FAL Node (Start after 3s - wait for MAVROS connection)
    # ========================================================================
    fal_node = TimerAction(
        period=3.0,
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
    
    # ========================================================================
    # TEE Node (Start after 6s - wait for FAL initialization)
    # ========================================================================
    tee_node = TimerAction(
        period=6.0,
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
    """Generate launch description for complete single-drone system."""
    
    # ========================================================================
    # Launch Arguments
    # ========================================================================
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID (0-based). Determines namespace, ports, and system ID.'
    )
    
    connection_protocol_arg = DeclareLaunchArgument(
        'connection_protocol',
        default_value='tcp',
        description='Protocol to use for MAVROS connection (tcp/udp)'
    )
    
    sitl_port_arg = DeclareLaunchArgument(
        'sitl_port',
        default_value='14550',
        description='Base port for SITL connection (14550 for tcpin MAVProxy)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug/info/warn/error)'
    )
    
    use_docker_arg = DeclareLaunchArgument(
        'use_docker',
        default_value='true',
        description='Use Docker networking (TCP to sitl_droneN) or native UDP'
    )
    
    fcu_host_arg = DeclareLaunchArgument(
        'fcu_host',
        default_value='host.docker.internal',
        description='FCU host address (host.docker.internal for host SITL, or sitl_droneN for container SITL)'
    )
    
    return LaunchDescription([
        # Arguments
        drone_id_arg,
        connection_protocol_arg,
        sitl_port_arg,
        log_level_arg,
        use_docker_arg,
        fcu_host_arg,
        
        # Generate drone nodes dynamically
        OpaqueFunction(function=generate_drone_nodes),
    ])