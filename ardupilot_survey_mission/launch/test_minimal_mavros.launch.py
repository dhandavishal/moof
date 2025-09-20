#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://:14555@127.0.0.1:14556',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                # ArduPilot-specific system settings
                'heartbeat_mav_type': 'ONBOARD_CONTROLLER',
                'conn_timeout': 10.0,
                'timesync_mode': 'MAVLINK',
                # ONLY these plugins - everything else is blocked
                'plugin_allowlist': [
                    'sys_status',
                    'command',
                    'global_position',
                    'guided_target',  # Added for waypoint navigation
                    'local_position', # Added for better position feedback
                ],
                # Don't use denylist at all - just allowlist is more restrictive
            }],
        )
    ])
