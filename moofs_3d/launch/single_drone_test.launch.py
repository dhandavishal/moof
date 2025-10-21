#!/usr/bin/env python3
"""Launch file for complete single-drone system test.

This launch file starts:
- SITL (must be started separately)
- MAVROS
- FAL Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for single drone test."""
    
    # Get package directories
    moofs_3d_dir = get_package_share_directory('moofs_3d')
    fal_dir = get_package_share_directory('flight_abstraction')
    
    # Declare launch arguments
    drone_id_arg = DeclareLaunchArgument(
        'drone_id',
        default_value='0',
        description='Drone ID (e.g., 0 for /drone_0)'
    )
    
    drone_id = LaunchConfiguration('drone_id')
    
    # Include MAVROS launch (for single drone)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moofs_3d_dir, 'launch', 'multi_mavros.launch.py')
        ),
        launch_arguments={
            'num_drones': '1',
            'start_id': drone_id,
        }.items()
    )
    
    # Include FAL launch
    fal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fal_dir, 'launch', 'fal.launch.py')
        ),
        launch_arguments={
            'drone_namespace': ['/drone_', drone_id],
        }.items()
    )
    
    return LaunchDescription([
        drone_id_arg,
        mavros_launch,
        fal_launch,
    ])
