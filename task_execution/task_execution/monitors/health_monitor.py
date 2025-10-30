#!/usr/bin/env python3
"""
Unified health monitoring system for ROS2.

Aggregates battery, GPS, and other health monitors into a single
comprehensive health assessment system.
"""

from typing import Dict, Optional, Callable

from rclpy.node import Node
from task_execution.monitors.battery_monitor import BatteryMonitor
from task_execution.monitors.gps_monitor import GPSMonitor
from multi_drone_msgs.msg import HealthReport


class HealthMonitor:
    """
    Unified health monitoring system.
    Aggregates battery, GPS, and other health monitors.
    """
    
    def __init__(self, node: Node, config: dict, namespace: str = ''):
        """
        Initialize unified health monitor.
        
        Args:
            node: ROS2 node instance
            config: Configuration dictionary
            namespace: Drone namespace
        """
        self.node = node
        self.config = config
        self.namespace = namespace
        
        # Component monitors
        self.battery_monitor = BatteryMonitor(node, config, namespace)
        self.gps_monitor = GPSMonitor(node, config, namespace)
        
        # Overall health state
        self.overall_health = 'unknown'
        
        # Emergency callback (triggered on critical health)
        self.emergency_callback: Optional[Callable] = None
        
        # Register critical callbacks from sub-monitors
        self.battery_monitor.register_critical_callback(self._on_battery_critical)
        self.gps_monitor.register_critical_callback(self._on_gps_critical)
        
        # Publisher for overall health
        health_topic = f'/{namespace}/tee/health/overall' if namespace else '/tee/health/overall'
        self.health_pub = self.node.create_publisher(
            HealthReport,
            health_topic,
            10
        )
        
        self.node.get_logger().info("Unified health monitor initialized")
    
    def start_monitoring(self):
        """Start all health monitors"""
        self.battery_monitor.start_monitoring()
        self.gps_monitor.start_monitoring()
        self.node.get_logger().info("All health monitoring started")
    
    def stop_monitoring(self):
        """Stop all health monitors"""
        self.battery_monitor.stop_monitoring()
        self.gps_monitor.stop_monitoring()
        self.node.get_logger().info("All health monitoring stopped")
    
    def get_overall_health(self) -> str:
        """
        Calculate overall system health.
        
        Returns:
            Overall health state: 'good', 'warning', 'critical', 'unknown'
        """
        battery_status = self.battery_monitor.get_current_status()
        gps_status = self.gps_monitor.get_current_status()
        
        if not battery_status or not gps_status:
            return 'unknown'
        
        # If any component is critical, overall is critical
        if (battery_status.health_state == 'critical' or 
            gps_status.health_state == 'critical'):
            self.overall_health = 'critical'
        
        # If any component is warning and none critical, overall is warning
        elif (battery_status.health_state == 'warning' or 
              gps_status.health_state == 'warning'):
            self.overall_health = 'warning'
        
        # If all good, overall is good
        elif (battery_status.health_state == 'good' and 
              gps_status.health_state == 'good'):
            self.overall_health = 'good'
        
        else:
            self.overall_health = 'unknown'
        
        # Publish overall health
        self._publish_overall_health()
        
        return self.overall_health
    
    def _publish_overall_health(self):
        """Publish overall health status"""
        report = HealthReport()
        report.component = 'overall'
        report.state = self.overall_health
        report.value = 0.0
        report.message = f"Overall system health: {self.overall_health}"
        report.timestamp = self.node.get_clock().now().to_msg()
        
        self.health_pub.publish(report)
    
    def _on_battery_critical(self, battery_status):
        """Handle critical battery state"""
        self.node.get_logger().error("CRITICAL: Battery health critical")
        if self.emergency_callback:
            self.emergency_callback('battery_critical', battery_status)
    
    def _on_gps_critical(self, gps_status):
        """Handle critical GPS state"""
        self.node.get_logger().error("CRITICAL: GPS health critical")
        if self.emergency_callback:
            self.emergency_callback('gps_critical', gps_status)
    
    def register_emergency_callback(self, callback: Callable):
        """
        Register callback for emergency situations.
        
        Args:
            callback: Function(reason: str, status: object)
        """
        self.emergency_callback = callback
    
    def get_health_summary(self) -> Dict:
        """
        Get comprehensive health summary.
        
        Returns:
            Dictionary with all health information
        """
        battery_status = self.battery_monitor.get_current_status()
        gps_status = self.gps_monitor.get_current_status()
        
        summary = {
            'overall': self.overall_health,
            'battery': {
                'state': battery_status.health_state if battery_status else 'unknown',
                'percentage': battery_status.percentage if battery_status else 0.0,
                'voltage': battery_status.voltage if battery_status else 0.0,
                'current': battery_status.current if battery_status else 0.0
            },
            'gps': {
                'state': gps_status.health_state if gps_status else 'unknown',
                'satellites': gps_status.satellites if gps_status else 0,
                'hdop': gps_status.hdop if gps_status else 99.0,
                'fix_type': gps_status.fix_type if gps_status else 0
            }
        }
        
        return summary
