#!/usr/bin/env python3
"""
Multi-drone system launch file with Squadron Manager.

Launches:
- Multiple drone instances (each with MAVROS, TF, FAL, TEE)
- Squadron Manager for coordination

Prerequisites:
- Multiple ArduPilot SITL instances running on different ports
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_drone_nodes(context, *args, **kwargs):
    """Generate nodes for each drone instance"""
    
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    log_level = LaunchConfiguration('log_level')
    
    # MAVROS port mapping for multiple SITL instances
    # Drone 0: 14550/14555 (out/in)
    # Drone 1: 14560/14565
    # Drone 2: 14570/14575
    # etc.
    
    drone_nodes = []
    mavros_dir = get_package_share_directory('mavros')
    
    for i in range(num_drones):
        drone_id = str(i)
        drone_namespace = f'/drone_{i}'
        
        # Calculate MAVROS ports
        fcu_port = 14550 + (i * 10)  # Output port
        gcs_port = 14555 + (i * 10)  # Input port
        fcu_url = f'udp://:{fcu_port}@127.0.0.1:{gcs_port}'
        tgt_system = str(i + 1)
        
        # TF frame names
        odom_frame = f'drone_{i}/odom'
        
        # ====================================================================
        # TF Transform: map -> odom for this drone
        # ====================================================================
        tf_map_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'map_to_odom_broadcaster_{i}',
            arguments=['0', '0', '0', '0', '0', '0', 'map', odom_frame],
            output='screen'
        )
        
        # ====================================================================
        # MAVROS Node for this drone
        # ====================================================================
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
        
        # ====================================================================
        # FAL Node for this drone (with delay)
        # ====================================================================
        fal_node = TimerAction(
            period=3.0 + i * 0.5,  # Stagger startup
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
        
        # ====================================================================
        # TEE Node for this drone (with delay)
        # ====================================================================
        tee_config_file = PathJoinSubstitution([
            FindPackageShare('task_execution'),
            'config',
            'tee_config.yaml'
        ])
        
        tee_node = TimerAction(
            period=6.0 + i * 0.5,  # Stagger startup
            actions=[
                Node(
                    package='task_execution',
                    executable='tee_node',
                    name='task_execution_engine',
                    namespace=drone_namespace,
                    output='screen',
                    parameters=[{
                        'config_path': tee_config_file,
                    }],
                    arguments=['--ros-args', '--log-level', log_level]
                )
            ]
        )
        
        # Add all nodes for this drone
        drone_nodes.extend([
            tf_map_to_odom,
            mavros_group,
            fal_node,
            tee_node
        ])
    
    return drone_nodes


def generate_launch_description():
    """Generate launch description for multi-drone squadron system."""
    
    # Declare arguments
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones in squadron (1-10)'
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
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug/info/warn/error)'
    )
    
    # Get launch configurations
    num_drones = LaunchConfiguration('num_drones')
    allocation_strategy = LaunchConfiguration('allocation_strategy')
    enable_formations = LaunchConfiguration('enable_formations')
    log_level = LaunchConfiguration('log_level')
    
    # ========================================================================
    # Squadron Manager Node (launches after drones are up)
    # ========================================================================
    squadron_config_file = PathJoinSubstitution([
        FindPackageShare('squadron_manager'),
        'config',
        'squadron_config.yaml'
    ])
    
    squadron_manager_node = TimerAction(
        period=10.0,  # Wait for all drones to initialize
        actions=[
            Node(
                package='squadron_manager',
                executable='squadron_manager_node',
                name='squadron_manager',
                output='screen',
                parameters=[
                    squadron_config_file,
                    {
                        'num_drones': num_drones,
                        'allocation_strategy': allocation_strategy,
                        'enable_formations': enable_formations,
                    }
                ],
                arguments=['--ros-args', '--log-level', log_level]
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        num_drones_arg,
        allocation_strategy_arg,
        enable_formations_arg,
        log_level_arg,
        
        # Generate drone nodes dynamically
        OpaqueFunction(function=generate_drone_nodes),
        
        # Squadron Manager
        squadron_manager_node,
    ])
