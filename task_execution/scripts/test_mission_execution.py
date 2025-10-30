#!/usr/bin/env python3
"""
Integration test for complete mission execution.
Tests TEE with simulated mission commands.
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionTestSender(Node):
    """Send test missions to TEE"""
    
    def __init__(self):
        super().__init__('mission_test_sender')
        
        # Create publisher
        self.mission_pub = self.create_publisher(
            String,
            '/squadron/mission_command',
            10
        )
        
        self.get_logger().info("Mission test sender initialized")
    
    def send_test_missions(self):
        """Send test missions"""
        
        # Wait for TEE to be ready
        self.get_logger().info("Waiting for TEE to be ready...")
        time.sleep(3.0)
        
        # Test Mission 1: Simple waypoint navigation
        self.get_logger().info("Sending waypoint mission...")
        
        waypoint_mission = {
            'mission_id': 'test_waypoint_001',
            'task_type': 'waypoint',
            'priority': 50,
            'timeout': 300.0,
            'parameters': {
                'waypoints': [
                    {'position': [0.0, 0.0, 50.0]},
                    {'position': [50.0, 0.0, 50.0]},
                    {'position': [50.0, 50.0, 50.0]},
                    {'position': [0.0, 50.0, 50.0]},
                    {'position': [0.0, 0.0, 50.0]}
                ],
                'velocity': 10.0
            }
        }
        
        msg = String()
        msg.data = json.dumps(waypoint_mission)
        self.mission_pub.publish(msg)
        self.get_logger().info("Waypoint mission sent")
        
        # Wait before sending next mission
        time.sleep(2.0)
        
        # Test Mission 2: Survey mission
        self.get_logger().info("Sending survey mission...")
        
        survey_mission = {
            'mission_id': 'test_survey_001',
            'task_type': 'survey',
            'priority': 50,
            'timeout': 600.0,
            'parameters': {
                'area': [
                    [100.0, 0.0],
                    [200.0, 0.0],
                    [200.0, 100.0],
                    [100.0, 100.0]
                ],
                'altitude': 50.0,
                'overlap_forward': 0.75,
                'overlap_side': 0.65
            }
        }
        
        msg = String()
        msg.data = json.dumps(survey_mission)
        self.mission_pub.publish(msg)
        self.get_logger().info("Survey mission sent")
        
        # Wait before sending next mission
        time.sleep(2.0)
        
        # Test Mission 3: Search mission
        self.get_logger().info("Sending search mission...")
        
        search_mission = {
            'mission_id': 'test_search_001',
            'task_type': 'search',
            'priority': 50,
            'timeout': 600.0,
            'parameters': {
                'center': [150.0, 50.0],
                'radius': 30.0,
                'pattern': 'expanding_square',
                'altitude': 30.0
            }
        }
        
        msg = String()
        msg.data = json.dumps(search_mission)
        self.mission_pub.publish(msg)
        self.get_logger().info("Search mission sent")
        
        self.get_logger().info("All test missions sent successfully")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    sender = MissionTestSender()
    
    try:
        sender.send_test_missions()
        
        # Keep node alive for a bit
        time.sleep(5.0)
        
    except KeyboardInterrupt:
        sender.get_logger().info("Interrupted")
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
