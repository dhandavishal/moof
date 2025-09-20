#!/usr/bin/env python3

import os
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
                # Only load absolutely essential plugins
                'plugin_allowlist': [
                    'sys_status',
                    'command',
                    'global_position',
                    'guided_target',
                ],
                # Explicitly deny problematic plugins
                'plugin_denylist': [
                    'debug_value',
                    'companion_process_status',
                    'setpoint_*',
                    'actuator_control',
                    'adsb',
                    'altitude',
                    'cam_imu_sync',
                    'camera',
                    'cellular_status',
                ],
            }],
        )
    ])
