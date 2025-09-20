#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            namespace='drone0',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14550@127.0.0.1:14551',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                # Minimal plugin list to avoid conflicts
                'plugin_allowlist': [
                    'sys_*',
                    'command',
                    'param',
                    'global_position',
                    'local_position',
                    'home_position',
                    'imu',
                    'guided_target',  # Essential for ArduPilot
                ],
                'plugin_denylist': [
                    'setpoint_*',  # Disable all setpoint plugins
                    'companion_process_status',  # Disable problematic plugin
                ],
            }],
        )
    ])