#!/usr/bin/env python3
"""
TF Diagnostic Script for AeroStack2 MAVROS Integration
Helps diagnose TF extrapolation and timing issues.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import time

class TFDiagnostic(Node):
    def __init__(self):
        super().__init__('tf_diagnostic')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for periodic diagnostics
        self.timer = self.create_timer(2.0, self.run_diagnostics)
        
        self.get_logger().info("TF Diagnostic node started")
        
    def run_diagnostics(self):
        """Run TF diagnostics"""
        try:
            # Test common frame transforms (updated with correct namespaced frames)
            frames_to_test = [
                ('earth', 'drone0/base_link'),  # Main transform platform needs
                ('drone0/odom', 'drone0/base_link'),  # Odometry transform
                ('earth', 'drone0/odom'),       # Global to odom transform
                ('drone0/map', 'drone0/base_link'),  # Map transform (if exists)
            ]
            
            for source_frame, target_frame in frames_to_test:
                try:
                    # Try to get latest transform
                    transform = self.tf_buffer.lookup_transform(
                        target_frame, 
                        source_frame, 
                        rclpy.time.Time(),  # Latest available
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # Calculate time difference
                    now = self.get_clock().now()
                    transform_time = transform.header.stamp
                    time_diff = (now - rclpy.time.Time.from_msg(transform_time)).nanoseconds / 1e9
                    
                    self.get_logger().info(
                        f"✓ Transform {source_frame} -> {target_frame}: "
                        f"age={time_diff:.3f}s"
                    )
                    
                except Exception as e:
                    self.get_logger().warn(
                        f"✗ Transform {source_frame} -> {target_frame}: {str(e)}"
                    )
                    
        except Exception as e:
            self.get_logger().error(f"Diagnostic error: {e}")

def main():
    rclpy.init()
    
    try:
        node = TFDiagnostic()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
