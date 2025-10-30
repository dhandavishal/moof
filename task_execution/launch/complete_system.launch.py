#!/usr/bin/env python3
"""
Complete single-drone system launch file.

Launches:
- MAVROS (connects to ArduPilot SITL)
- FAL (Flight Abstraction Layer)
- TEE (Task Execution Engine)

Prerequisites:
- ArduPilot SITL must be running on ports 14550/14555
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for complete single-drone system."""
    
    # Declare arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID (0-based)'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@127.0.0.1:14555',
        description='MAVROS FCU URL for ArduPilot SITL'
    )
    
    tgt_system_arg = DeclareLaunchArgument(
        'tgt_system',
        default_value='1',
        description='Target system ID'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug/info/warn/error)'
    )
    
    # Get launch configurations
    drone_id = LaunchConfiguration('drone_id')
    fcu_url = LaunchConfiguration('fcu_url')
    tgt_system = LaunchConfiguration('tgt_system')
    log_level = LaunchConfiguration('log_level')
    
    # Drone namespace
    drone_namespace = ['/drone_', drone_id]
    
    # ========================================================================
    # MAVROS Node
    # ========================================================================
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace=drone_namespace,
        output='screen',
        parameters=[{
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': tgt_system,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
            'system_id': 255,
            'component_id': 240,
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )
    
    # ========================================================================
    # FAL Node (with delay to wait for MAVROS)
    # ========================================================================
    fal_node = TimerAction(
        period=3.0,  # Wait 3 seconds for MAVROS to connect
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
    
    # ========================================================================
    # TEE Node (with delay to wait for FAL)
    # ========================================================================
    tee_config_file = PathJoinSubstitution([
        FindPackageShare('task_execution'),
        'config',
        'tee_config.yaml'
    ])
    
    tee_node = TimerAction(
        period=6.0,  # Wait 6 seconds for MAVROS and FAL to initialize
        actions=[
            Node(
                package='task_execution',
                executable='tee_node',
                name='task_execution_engine',
                output='screen',
                parameters=[{
                    'config_path': tee_config_file,
                }],
                arguments=['--ros-args', '--log-level', log_level]
            )
        ]
    )
    
    # ========================================================================
    # Status Monitor (Optional - with delay)
    # ========================================================================
    status_monitor = TimerAction(
        period=10.0,  # Wait 10 seconds for everything to initialize
        actions=[
            Node(
                package='task_execution',
                executable='monitor_tee_status',
                name='tee_status_monitor',
                output='screen',
                arguments=['--ros-args', '--log-level', 'warn']
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        drone_id_arg,
        fcu_url_arg,
        tgt_system_arg,
        log_level_arg,
        
        # Nodes (sequential startup)
        mavros_node,      # Start immediately
        fal_node,         # Start after 3s
        tee_node,         # Start after 6s
        # status_monitor, # Uncomment to enable automatic status monitoring
    ])
