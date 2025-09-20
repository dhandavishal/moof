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
                'fcu_url': 'udp://:14550@127.0.0.1:14551',
                'gcs_url': '',
                'target_system_id': 1,
                'target_component_id': 1,
                # ArduPilot-specific system settings
                'heartbeat_mav_type': 'ONBOARD_CONTROLLER',
                'conn_timeout': 10.0,
                'timesync_mode': 'MAVLINK',
                # Essential plugins for basic flight operations
                'plugin_allowlist': [
                    'sys_status',
                    'command',
                    'global_position',
                    'local_position',
                    'guided_target',  # Essential for ArduPilot GUIDED mode
                ],
                # Official ArduPilot plugin denylist from wiki
                'plugin_denylist': [
                    'actuator_control',
                    'ftp',
                    'hil',
                    'altitude',
                    'debug_value',
                    'image_pub',
                    'px4flow',
                    'vibration',
                    'vision_speed_estimate',
                    'wheel_odometry',
                    'companion_process_status',  # Additional problematic plugin
                ],
            }],
        )
    ])
