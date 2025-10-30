#!/usr/bin/env python3
"""
GPS health monitor for ROS2.

Monitors GPS health and quality, detects fix loss, poor accuracy,
and position jumps.
"""

import threading
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, Callable, List
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from multi_drone_msgs.msg import HealthReport


@dataclass
class GPSStatus:
    """GPS status data structure"""
    latitude: float
    longitude: float
    altitude: float
    fix_type: int  # 0=no fix, 1=2D, 2=3D, 3=RTK
    satellites: int
    hdop: float
    vdop: float
    health_state: str
    timestamp: float


class GPSMonitor:
    """
    Monitors GPS health and quality.
    Detects fix loss, poor accuracy, and position jumps.
    """
    
    def __init__(self, node: Node, config: dict, namespace: str = ''):
        """
        Initialize GPS monitor.
        
        Args:
            node: ROS2 node instance
            config: Configuration dictionary
            namespace: Drone namespace
        """
        self.node = node
        self.config = config
        self.namespace = namespace
        
        safety = config.get('safety', {})
        
        # Thresholds from config
        self.min_satellites = safety.get('min_gps_satellites', 8)
        self.max_hdop = safety.get('max_gps_hdop', 2.0)
        self.max_position_jump = 10.0  # meters
        
        # Current status
        self.current_status: Optional[GPSStatus] = None
        
        # Historical data
        self.position_history = deque(maxlen=50)  # Last 5 seconds at 10Hz
        self.hdop_history = deque(maxlen=50)
        
        # Last valid position for jump detection
        self.last_position = None
        self.last_timestamp = None
        
        # Callbacks
        self.warning_callbacks: List[Callable] = []
        self.critical_callbacks: List[Callable] = []
        
        # Monitoring
        self.monitoring = False
        self.monitor_thread = None
        
        # QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ROS communication
        gps_topic = f'/{namespace}/mavros/global_position/global' if namespace else '/mavros/global_position/global'
        health_topic = f'/{namespace}/tee/health/gps' if namespace else '/tee/health/gps'
        
        self.gps_sub = self.node.create_subscription(
            NavSatFix,
            gps_topic,
            self._gps_callback,
            qos_profile
        )
        
        self.health_pub = self.node.create_publisher(
            HealthReport,
            health_topic,
            10
        )
        
        self.node.get_logger().info("GPS monitor initialized")
    
    def start_monitoring(self):
        """Start continuous monitoring"""
        if self.monitoring:
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True
        )
        self.monitor_thread.start()
        self.node.get_logger().info("GPS monitoring started")
    
    def stop_monitoring(self):
        """Stop monitoring"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        self.node.get_logger().info("GPS monitoring stopped")
    
    def _gps_callback(self, msg: NavSatFix):
        """
        Process incoming GPS fix messages.
        
        Args:
            msg: NavSatFix message
        """
        # Extract HDOP from covariance matrix
        hdop = np.sqrt(msg.position_covariance[0]) if msg.position_covariance[0] > 0 else 99.0
        vdop = np.sqrt(msg.position_covariance[8]) if msg.position_covariance[8] > 0 else 99.0
        
        # Get satellite count - estimate from fix quality
        if msg.status.status >= 2:  # 3D fix or better
            satellites = 10  # Placeholder - actual count from GPS status topic
        elif msg.status.status == 1:
            satellites = 6
        else:
            satellites = 0
        
        # Create status
        self.current_status = GPSStatus(
            latitude=msg.latitude,
            longitude=msg.longitude,
            altitude=msg.altitude,
            fix_type=msg.status.status,
            satellites=satellites,
            hdop=hdop,
            vdop=vdop,
            health_state='unknown',
            timestamp=self.node.get_clock().now().nanoseconds / 1e9
        )
        
        # Store position history
        self.position_history.append([msg.latitude, msg.longitude, msg.altitude])
        self.hdop_history.append(hdop)
        
        # Detect position jumps
        if self.last_position and self.last_timestamp:
            self._check_position_jump(msg)
        
        self.last_position = [msg.latitude, msg.longitude, msg.altitude]
        self.last_timestamp = msg.header.stamp
    
    def _monitoring_loop(self):
        """Main monitoring loop"""
        import time
        
        while self.monitoring:
            if self.current_status is None:
                time.sleep(0.1)
                continue
            
            # Assess GPS health
            health_state = self._assess_gps_health()
            self.current_status.health_state = health_state
            
            # Publish health report
            self._publish_health_report()
            
            # Trigger callbacks
            if health_state == 'critical':
                self._trigger_critical_callbacks()
            elif health_state == 'warning':
                self._trigger_warning_callbacks()
            
            time.sleep(0.1)  # 10Hz
    
    def _assess_gps_health(self) -> str:
        """
        Assess GPS health based on multiple factors.
        
        Returns:
            Health state: 'good', 'warning', 'critical'
        """
        if not self.current_status:
            return 'unknown'
        
        status = self.current_status
        
        # Level 1: No fix (critical)
        if status.fix_type < 1:
            self.node.get_logger().error("CRITICAL: GPS has no fix")
            return 'critical'
        
        # Level 2: Only 2D fix (warning)
        if status.fix_type == 1:
            self.node.get_logger().warn("WARNING: GPS only has 2D fix")
            return 'warning'
        
        # Level 3: Insufficient satellites (critical)
        if status.satellites < 6:
            self.node.get_logger().error(f"CRITICAL: Only {status.satellites} satellites")
            return 'critical'
        elif status.satellites < self.min_satellites:
            self.node.get_logger().warn(f"WARNING: Low satellite count: {status.satellites}")
            return 'warning'
        
        # Level 4: Poor accuracy (HDOP)
        if status.hdop > 5.0:
            self.node.get_logger().error(f"CRITICAL: HDOP too high: {status.hdop:.2f}")
            return 'critical'
        elif status.hdop > self.max_hdop:
            self.node.get_logger().warn(f"WARNING: Elevated HDOP: {status.hdop:.2f}")
            return 'warning'
        
        # Level 5: HDOP degradation trend
        if len(self.hdop_history) >= 20:
            recent_hdop = list(self.hdop_history)[-20:]
            avg_hdop = np.mean(recent_hdop)
            if avg_hdop > self.max_hdop * 1.5:
                self.node.get_logger().warn("WARNING: HDOP degradation trend")
                return 'warning'
        
        return 'good'
    
    def _check_position_jump(self, msg: NavSatFix):
        """
        Check for unrealistic position jumps indicating GPS glitch.
        
        Args:
            msg: Current GPS message
        """
        if not self.last_position or not self.last_timestamp:
            return
        
        # Calculate distance moved
        distance = self._calculate_distance(
            self.last_position[0], self.last_position[1],
            msg.latitude, msg.longitude
        )
        
        # Calculate time delta
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        last_time = self.last_timestamp.sec + self.last_timestamp.nanosec / 1e9
        time_delta = current_time - last_time
        
        if time_delta > 0:
            velocity = distance / time_delta
            
            # Check for unrealistic velocity (e.g., >50 m/s = 180 km/h)
            max_realistic_velocity = 50.0  # m/s
            
            if velocity > max_realistic_velocity:
                self.node.get_logger().warn(
                    f"GPS position jump detected: {distance:.1f}m in {time_delta:.2f}s "
                    f"({velocity:.1f}m/s)"
                )
    
    def _calculate_distance(self, lat1, lon1, lat2, lon2) -> float:
        """
        Calculate distance between two GPS coordinates using Haversine formula.
        
        Args:
            lat1, lon1: First coordinate
            lat2, lon2: Second coordinate
            
        Returns:
            Distance in meters
        """
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat / 2) ** 2 +
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2)
        
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        distance = R * c
        return distance
    
    def _publish_health_report(self):
        """Publish GPS health report"""
        if not self.current_status:
            return
        
        report = HealthReport()
        report.component = 'gps'
        report.state = self.current_status.health_state
        report.value = float(self.current_status.satellites)
        report.message = (
            f"GPS: {self.current_status.satellites} sats, "
            f"HDOP {self.current_status.hdop:.2f}, "
            f"Fix type {self.current_status.fix_type}"
        )
        report.timestamp = self.node.get_clock().now().to_msg()
        
        self.health_pub.publish(report)
    
    def register_warning_callback(self, callback: Callable):
        """Register warning callback"""
        self.warning_callbacks.append(callback)
    
    def register_critical_callback(self, callback: Callable):
        """Register critical callback"""
        self.critical_callbacks.append(callback)
    
    def _trigger_warning_callbacks(self):
        """Execute warning callbacks"""
        for callback in self.warning_callbacks:
            try:
                callback(self.current_status)
            except Exception as e:
                self.node.get_logger().error(f"Warning callback failed: {e}")
    
    def _trigger_critical_callbacks(self):
        """Execute critical callbacks"""
        for callback in self.critical_callbacks:
            try:
                callback(self.current_status)
            except Exception as e:
                self.node.get_logger().error(f"Critical callback failed: {e}")
    
    def get_current_status(self) -> Optional[GPSStatus]:
        """Get current GPS status"""
        return self.current_status
