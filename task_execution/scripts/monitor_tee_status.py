#!/usr/bin/env python3
"""
Monitor and display TEE status in real-time.
"""

import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TEEStatusMonitor(Node):
    """Display TEE status in terminal"""
    
    def __init__(self):
        super().__init__('tee_status_monitor')
        
        self.mission_status = None
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/tee/mission_status',
            self._status_callback,
            10
        )
        
        # Create timer for display updates (2Hz)
        self.display_timer = self.create_timer(0.5, self.display_status)
        
        self.get_logger().info("TEE Status Monitor started")
    
    def _status_callback(self, msg: String):
        """Update mission status"""
        try:
            self.mission_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse status JSON")
    
    def display_status(self):
        """Display status in terminal"""
        
        # Clear screen
        os.system('clear')
        
        print("=" * 60)
        print("TASK EXECUTION ENGINE STATUS")
        print("=" * 60)
        print()
        
        # Mission status
        if self.mission_status:
            print(f"State: {self.mission_status.get('state', 'UNKNOWN').upper()}")
            
            mission_id = self.mission_status.get('mission_id')
            if mission_id:
                print(f"Mission ID: {mission_id}")
            
            progress = self.mission_status.get('progress', 0.0)
            print(f"Progress: {progress*100:.1f}%")
            
            # Health
            health = self.mission_status.get('health', 'unknown')
            print(f"Health: {health.upper()}")
            
            timestamp = self.mission_status.get('timestamp', 0.0)
            print(f"Last Update: {timestamp:.2f}")
        else:
            print("Waiting for status updates...")
        
        print()
        print("-" * 60)
        print("Press Ctrl+C to exit")
    
    def run(self):
        """Run the monitor"""
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            self.get_logger().info("Monitor stopped")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    monitor = TEEStatusMonitor()
    
    try:
        monitor.run()
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
