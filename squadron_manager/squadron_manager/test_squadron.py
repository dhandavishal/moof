#!/usr/bin/env python3
"""
Test script for Squadron Manager
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SquadronTester(Node):
    """Test node for squadron manager"""
    
    def __init__(self):
        super().__init__('squadron_tester')
        
        self.mission_pub = self.create_publisher(
            String,
            '/squadron/mission_command',
            10
        )
        
        self.formation_pub = self.create_publisher(
            String,
            '/squadron/formation_command',
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/squadron/status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("Squadron Tester initialized")
        
        # Wait for publishers to connect
        time.sleep(2.0)
    
    def status_callback(self, msg):
        """Handle squadron status"""
        try:
            status = json.loads(msg.data)
            self.get_logger().info(
                f"Squadron Status: {status['available']}/{status['total_drones']} available, "
                f"{status['busy']} busy, {status['average_battery']:.1f}% avg battery"
            )
        except json.JSONDecodeError:
            pass
    
    def send_single_drone_mission(self):
        """Send a simple single-drone waypoint mission"""
        mission = {
            "mission_id": f"test_single_{int(time.time())}",
            "task_type": "waypoint",
            "priority": 100,
            "timeout": 120.0,
            "multi_drone": False,
            "parameters": {
                "waypoints": [
                    {"x": 20.0, "y": 20.0, "z": 50.0, "yaw": 0.0}
                ],
                "velocity": 3.0,
                "acceptance_radius": 1.0
            }
        }
        
        msg = String()
        msg.data = json.dumps(mission)
        self.mission_pub.publish(msg)
        
        self.get_logger().info(f"Sent single-drone mission: {mission['mission_id']}")
    
    def send_multi_drone_mission(self):
        """Send a multi-drone formation mission"""
        mission = {
            "mission_id": f"test_formation_{int(time.time())}",
            "task_type": "waypoint",
            "priority": 100,
            "timeout": 180.0,
            "multi_drone": True,
            "formation_type": "wedge",
            "spacing": 15.0,
            "altitude": 50.0,
            "parameters": {
                "waypoints": [
                    {"x": 50.0, "y": 0.0, "z": 50.0, "yaw": 0.0},
                    {"x": 50.0, "y": 50.0, "z": 50.0, "yaw": 1.57}
                ],
                "velocity": 3.0
            }
        }
        
        msg = String()
        msg.data = json.dumps(mission)
        self.mission_pub.publish(msg)
        
        self.get_logger().info(f"Sent multi-drone mission: {mission['mission_id']}")
    
    def create_line_formation(self):
        """Create a line formation"""
        command = {
            "type": "create",
            "formation_type": "line",
            "spacing": 10.0,
            "altitude": 50.0,
            "center": [0.0, 0.0, 50.0]
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.formation_pub.publish(msg)
        
        self.get_logger().info("Sent line formation command")
    
    def create_wedge_formation(self):
        """Create a wedge formation"""
        command = {
            "type": "create",
            "formation_type": "wedge",
            "spacing": 12.0,
            "altitude": 50.0,
            "center": [0.0, 0.0, 50.0]
        }
        
        msg = String()
        msg.data = json.dumps(command)
        self.formation_pub.publish(msg)
        
        self.get_logger().info("Sent wedge formation command")
    
    def break_formation(self):
        """Break current formation"""
        command = {"type": "break"}
        
        msg = String()
        msg.data = json.dumps(command)
        self.formation_pub.publish(msg)
        
        self.get_logger().info("Sent break formation command")
    
    def run_tests(self):
        """Run a series of tests"""
        self.get_logger().info("\n=== Starting Squadron Manager Tests ===\n")
        
        # Test 1: Single drone mission
        self.get_logger().info("Test 1: Single-drone waypoint mission")
        self.send_single_drone_mission()
        time.sleep(5.0)
        
        # Test 2: Create line formation
        self.get_logger().info("\nTest 2: Create line formation")
        self.create_line_formation()
        time.sleep(3.0)
        
        # Test 3: Create wedge formation
        self.get_logger().info("\nTest 3: Create wedge formation")
        self.create_wedge_formation()
        time.sleep(3.0)
        
        # Test 4: Multi-drone mission
        self.get_logger().info("\nTest 4: Multi-drone formation mission")
        self.send_multi_drone_mission()
        time.sleep(5.0)
        
        # Test 5: Break formation
        self.get_logger().info("\nTest 5: Break formation")
        self.break_formation()
        time.sleep(2.0)
        
        self.get_logger().info("\n=== Tests Complete ===\n")


def main():
    """Main entry point"""
    rclpy.init()
    
    tester = SquadronTester()
    
    # Run tests
    tester.run_tests()
    
    # Keep node alive to receive status updates
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
