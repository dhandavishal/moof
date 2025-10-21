#!/usr/bin/env python3
"""
Complete Multi-Drone System Launch
Launches MAVROS for multiple drones and monitoring node
Based on working basic_aerostack2 pattern
Note: SITL instances must be started separately
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_system_nodes(context, *args, **kwargs):
    """Generate all system nodes"""
    
    num_drones = int(LaunchConfiguration('num_drones').perform(context))
    use_state_estimator = LaunchConfiguration('use_state_estimator').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    base_fcu_port = 14550
    base_gcs_port = 14555
    
    mavros_dir = get_package_share_directory('mavros')
    
    actions = []
    
    # Generate MAVROS nodes for each drone
    for i in range(num_drones):
        drone_id = i
        namespace = f'drone_{drone_id}'
        
        # Calculate unique ports for this drone
        fcu_port = base_fcu_port + (drone_id * 10)
        gcs_port = base_gcs_port + (drone_id * 10)
        fcu_url = f'udp://:{gcs_port}@127.0.0.1:{fcu_port}'
        
        # MAVROS Launch wrapped in namespace group
        mavros_launch = GroupAction([
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
        
        # Static TF publisher for earth -> drone_X/odom
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'earth_to_{namespace}_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'earth', f'{namespace}/odom'],
            output='screen'
        )
        
        # Optional: State estimator for this drone
        if use_state_estimator.lower() == 'true':
            state_estimator = IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('as2_state_estimator'), 
                                'launch', 'state_estimator_launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'plugin_name': 'raw_odometry',
                    'use_sim_time': use_sim_time,
                }.items()
            )
            actions.append(state_estimator)
        
        actions.append(mavros_launch)
        actions.append(static_tf)
        
        print(f"Configured {namespace}:")
        print(f"  FCU URL: {fcu_url}")
        print(f"  Target System ID: {drone_id + 1}")
        print()
    
    # Add monitoring node with delay to allow MAVROS initialization
    monitor_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='moofs_3d',
                executable='multi_drone_monitor',
                name='multi_drone_monitor',
                output='screen',
                parameters=[{
                    'num_drones': num_drones,
                    'use_sim_time': use_sim_time
                }]
            )
        ]
    )
    actions.append(monitor_node)
    
    return actions


def generate_launch_description():
    """Generate complete launch description"""
    
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='3',
        description='Number of drones in the system'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_state_estimator_arg = DeclareLaunchArgument(
        'use_state_estimator',
        default_value='false',
        description='Use state estimator for each drone'
    )
    
    ld = LaunchDescription()
    ld.add_action(num_drones_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_state_estimator_arg)
    ld.add_action(OpaqueFunction(function=generate_system_nodes))
    
    return ld