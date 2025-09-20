#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix

class TestMAVROS(Node):
    def __init__(self):
        super().__init__('test_mavros')
        
        # Create subscriptions to test MAVROS
        self.state_sub = self.create_subscription(
            State, 
            '/drone0/mavros/state', 
            self.state_callback, 
            10
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/drone0/mavros/global_position/global',
            self.gps_callback,
            10
        )
        
        self.get_logger().info('Test MAVROS node started, waiting for messages...')
        
    def state_callback(self, msg):
        self.get_logger().info(f'State: Armed={msg.armed}, Mode={msg.mode}')
        
    def gps_callback(self, msg):
        self.get_logger().info(f'GPS: lat={msg.latitude:.6f}, lon={msg.longitude:.6f}')

def main(args=None):
    rclpy.init(args=args)
    node = TestMAVROS()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()