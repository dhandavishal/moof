#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ardupilot_survey_mission')
    
    # Declare arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone0',
        description='Namespace for the drone'
    )
    
    # Include MAVROS launch
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'mavros_sitl.launch.py')
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'fcu_url': 'udp://:14550@127.0.0.1:14551',
        }.items()
    )
    
    # Survey parameters file
    survey_params_file = os.path.join(pkg_dir, 'config', 'survey_params.yaml')
    
    # Survey mission node with proper delay
    survey_mission_node = TimerAction(
        period=5.0,  # Reduced to 5 seconds, should be enough for MAVROS
        actions=[
            Node(
                package='ardupilot_survey_mission',
                executable='survey_mission.py',
                name='survey_mission',
                namespace=LaunchConfiguration('namespace'),
                output='screen',
                parameters=[survey_params_file],
            )
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        mavros_launch,
        survey_mission_node,
    ])