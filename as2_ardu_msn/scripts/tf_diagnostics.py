#!/usr/bin/env python3
"""
TF Diagnostics Script for Aerostack2
Monitors transform health and timing issues
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros
import time


class TFDiagnostics(Node):
    def __init__(self):
        super().__init__('tf_diagnostics')
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribe to TF topics
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, 10)
        self.tf_static_sub = self.create_subscription(
            TFMessage, '/tf_static', self.tf_static_callback, 10)
        
        # Diagnostic timer
        self.timer = self.create_timer(2.0, self.diagnostic_callback)
        
        # Tracking variables
        self.tf_count = 0
        self.tf_static_count = 0
        self.last_earth_base_link = None
        self.transform_stats = {}
        
        self.get_logger().info("TF Diagnostics node started")

    def tf_callback(self, msg):
        """Monitor TF messages"""
        self.tf_count += 1
        current_time = self.get_clock().now()
        
        for transform in msg.transforms:
            frame_pair = f"{transform.header.frame_id} -> {transform.child_frame_id}"
            
            # Calculate timing delay
            msg_time = rclpy.time.Time.from_msg(transform.header.stamp)
            delay = (current_time - msg_time).nanoseconds / 1e9
            
            if frame_pair not in self.transform_stats:
                self.transform_stats[frame_pair] = {
                    'count': 0,
                    'avg_delay': 0.0,
                    'max_delay': 0.0,
                    'last_seen': current_time
                }
            
            stats = self.transform_stats[frame_pair]
            stats['count'] += 1
            stats['avg_delay'] = (stats['avg_delay'] * (stats['count'] - 1) + delay) / stats['count']
            stats['max_delay'] = max(stats['max_delay'], delay)
            stats['last_seen'] = current_time

    def tf_static_callback(self, msg):
        """Monitor static TF messages"""
        self.tf_static_count += 1

    def diagnostic_callback(self):
        """Print diagnostic information"""
        self.get_logger().info(f"=== TF Diagnostics ===")
        self.get_logger().info(f"TF messages received: {self.tf_count}")
        self.get_logger().info(f"TF static messages: {self.tf_static_count}")
        
        # Test specific transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'earth', 'drone0/base_link', rclpy.time.Time(), 
                timeout=rclpy.duration.Duration(seconds=0.1))
            
            current_time = self.get_clock().now()
            transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
            age = (current_time - transform_time).nanoseconds / 1e9
            
            self.get_logger().info(f"earth -> drone0/base_link: SUCCESS (age: {age:.3f}s)")
            
        except Exception as e:
            self.get_logger().warn(f"earth -> drone0/base_link: FAILED - {str(e)}")
        
        # Print top frame stats
        if self.transform_stats:
            self.get_logger().info("Top transform pairs by frequency:")
            sorted_stats = sorted(self.transform_stats.items(), 
                                key=lambda x: x[1]['count'], reverse=True)
            
            for frame_pair, stats in sorted_stats[:5]:
                self.get_logger().info(
                    f"  {frame_pair}: {stats['count']} msgs, "
                    f"avg_delay: {stats['avg_delay']:.3f}s, "
                    f"max_delay: {stats['max_delay']:.3f}s"
                )
        
        self.get_logger().info("=" * 25)


def main():
    rclpy.init()
    node = TFDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
