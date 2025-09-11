#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
import time

class SimpleArduPilotMission(Node):
    def __init__(self):
        super().__init__('simple_ardupilot_mission')
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Publisher for setpoints
        self.setpoint_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10
        )
        
        # State subscriber
        self.state = None
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        self.services_ready = False
        
    def wait_for_services(self):
        # Wait for services
        self.get_logger().info('Waiting for services...')
        
        # Wait for each service with timeout
        timeout_duration = 30.0
        if not self.arming_client.wait_for_service(timeout_sec=timeout_duration):
            self.get_logger().error('Arming service not available')
            return False
        if not self.mode_client.wait_for_service(timeout_sec=timeout_duration):
            self.get_logger().error('Set mode service not available')
            return False
        if not self.takeoff_client.wait_for_service(timeout_sec=timeout_duration):
            self.get_logger().error('Takeoff service not available')
            return False
            
        self.get_logger().info('All services available!')
        self.services_ready = True
        return True
        
    def state_callback(self, msg):
        self.state = msg
        
    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
        
    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().mode_sent
        
    def takeoff(self, altitude):
        req = CommandTOL.Request()
        req.altitude = altitude
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success
        
    def goto_position(self, x, y, z):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        # Send setpoint multiple times for better reliability
        for _ in range(200):  # Increased for better positioning
            self.setpoint_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.05)
            
    def run_mission(self):
        # Wait for services first
        if not self.wait_for_services():
            self.get_logger().error('Failed to connect to required services')
            return
        
        # Wait for connection
        while self.state is None or not self.state.connected:
            self.get_logger().info('Waiting for FCU connection...')
            rclpy.spin_once(self)
            time.sleep(1)
            
        self.get_logger().info('Connected to FCU!')
        
        # Set GUIDED mode
        self.get_logger().info('Setting GUIDED mode...')
        if self.set_mode('GUIDED'):
            self.get_logger().info('GUIDED mode set!')
        else:
            self.get_logger().error('Failed to set GUIDED mode')
            return
            
        # Arm
        self.get_logger().info('Arming...')
        if self.arm():
            self.get_logger().info('Armed!')
        else:
            self.get_logger().error('Failed to arm')
            return
            
        # Takeoff
        self.get_logger().info('Taking off to 5m...')
        if self.takeoff(5.0):
            self.get_logger().info('Takeoff command sent!')
        else:
            self.get_logger().error('Takeoff failed')
            return
            
        time.sleep(10)  # Wait for takeoff
        
        # Simple square pattern
        waypoints = [
            (5, 0, 5),
            (5, 5, 5),
            (0, 5, 5),
            (0, 0, 5)
        ]
        
        for i, (x, y, z) in enumerate(waypoints):
            self.get_logger().info(f'Going to waypoint {i+1}: ({x}, {y}, {z})')
            self.goto_position(x, y, z)
            time.sleep(5)
            
        self.get_logger().info('Mission complete!')

def main():
    rclpy.init()
    mission = SimpleArduPilotMission()
    mission.run_mission()
    rclpy.shutdown()

if __name__ == '__main__':
    main()