#!/usr/bin/env python3
"""
Monitor demonstration script.

Shows how to initialize and use health and progress monitors.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import yaml
import time

from task_execution.monitors.health_monitor import HealthMonitor
from task_execution.monitors.progress_monitor import ProgressMonitor


class MonitorDemo(Node):
    """Demonstration of monitor systems"""
    
    def __init__(self):
        super().__init__('monitor_demo')
        
        # Load configuration
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        if config_file:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
        else:
            # Default configuration
            config = {
                'safety': {
                    'min_battery_percentage': 0.25,
                    'critical_battery_percentage': 0.20,
                    'min_gps_satellites': 8,
                    'max_gps_hdop': 2.0
                },
                'drone': {
                    'battery_capacity': 5000,
                    'battery_voltage': 14.8
                }
            }
        
        self.get_logger().info("Initializing monitors...")
        
        # Initialize monitors
        self.health_monitor = HealthMonitor(self, config)
        self.progress_monitor = ProgressMonitor(self)
        
        # Register emergency callback
        self.health_monitor.register_emergency_callback(self._emergency_handler)
        
        # Start monitoring
        self.health_monitor.start_monitoring()
        
        # Create timer for health checks
        self.create_timer(5.0, self._check_health)
        
        self.get_logger().info("Monitor demo node started")
    
    def _check_health(self):
        """Periodic health check"""
        overall_health = self.health_monitor.get_overall_health()
        health_summary = self.health_monitor.get_health_summary()
        
        self.get_logger().info(f"Overall health: {overall_health}")
        self.get_logger().info(
            f"Battery: {health_summary['battery']['state']} - "
            f"{health_summary['battery']['percentage']*100:.1f}%"
        )
        self.get_logger().info(
            f"GPS: {health_summary['gps']['state']} - "
            f"{health_summary['gps']['satellites']} sats, "
            f"HDOP {health_summary['gps']['hdop']:.2f}"
        )
    
    def _emergency_handler(self, reason: str, status):
        """Handle emergency situations"""
        self.get_logger().error(f"EMERGENCY: {reason}")
        self.get_logger().error(f"Status: {status}")
        # In real implementation, trigger RTL or emergency landing
    
    def demonstrate_progress_monitor(self):
        """Demonstrate progress monitoring"""
        self.get_logger().info("=== Progress Monitor Demo ===")
        
        # Simulate waypoint mission
        task_params = {
            'waypoints': [
                {'position': [0, 0, 50]},
                {'position': [100, 0, 50]},
                {'position': [100, 100, 50]},
                {'position': [0, 100, 50]}
            ]
        }
        
        self.progress_monitor.start_mission(
            task_id='demo_waypoint_001',
            task_type='waypoint',
            task_parameters=task_params,
            total_primitives=4
        )
        
        # Simulate waypoint completion
        for i in range(4):
            time.sleep(2.0)
            self.progress_monitor.update_waypoint_reached(i)
            self.progress_monitor.update_primitive_completed(i)
            
            report = self.progress_monitor.get_progress_report()
            if report:
                self.get_logger().info(
                    f"Progress: {report.completion_percentage:.1f}%, "
                    f"Waypoint {report.waypoints_completed}/{report.waypoints_total}, "
                    f"ETA: {report.eta_seconds:.1f}s"
                )
        
        self.progress_monitor.complete_mission()
        self.get_logger().info("=== Demo Complete ===")
    
    def shutdown(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down monitors...")
        self.health_monitor.stop_monitoring()


def main(args=None):
    rclpy.init(args=args)
    
    node = MonitorDemo()
    
    # Create executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        # Run for a bit to collect health data
        print("\nMonitoring health for 10 seconds...")
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time < 10.0):
            executor.spin_once(timeout_sec=0.1)
        
        # Demonstrate progress monitoring
        node.demonstrate_progress_monitor()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
