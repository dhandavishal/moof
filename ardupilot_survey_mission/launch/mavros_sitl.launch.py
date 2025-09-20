#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ardupilot_survey_mission')
    
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone0',
        description='Namespace for the drone'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14550@127.0.0.1:14551',
        description='FCU connection URL for ArduPilot SITL'
    )
    
    # MAVROS configuration file
    mavros_config_file = os.path.join(pkg_dir, 'config', 'mavros_config.yaml')
    
    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            mavros_config_file,
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
            }
        ],
        remappings=[]
    )
    
    return LaunchDescription([
        namespace_arg,
        fcu_url_arg,
        mavros_node,
    ])