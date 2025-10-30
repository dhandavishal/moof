#!/usr/bin/env python3
"""
Battery health monitor for ROS2.

Monitors battery health with predictive alerting, tracking voltage, current,
percentage, temperature, and cell balance.
"""

import threading
from collections import deque
from dataclasses import dataclass
from typing import Optional, Callable, List
import numpy as np

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState
from multi_drone_msgs.msg import HealthReport


@dataclass
class BatteryStatus:
    """Battery status data structure"""
    voltage: float
    current: float
    percentage: float
    capacity_wh: float
    temperature: float
    cell_voltages: List[float]
    health_state: str  # 'good', 'warning', 'critical'
    timestamp: float


class BatteryMonitor:
    """
    Monitors battery health with predictive alerting.
    Tracks voltage, current, percentage, temperature, and cell balance.
    """
    
    def __init__(self, node: Node, config: dict, namespace: str = ''):
        """
        Initialize battery monitor.
        
        Args:
            node: ROS2 node instance
            config: Configuration dictionary
            namespace: Drone namespace (e.g., 'drone_1')
        """
        self.node = node
        self.config = config
        self.namespace = namespace
        
        safety = config.get('safety', {})
        drone = config.get('drone', {})
        
        # Thresholds from config
        self.min_battery_pct = safety.get('min_battery_percentage', 0.25)
        self.critical_battery_pct = safety.get('critical_battery_percentage', 0.20)
        self.battery_capacity_mah = drone.get('battery_capacity', 5000)
        self.battery_voltage_nominal = drone.get('battery_voltage', 14.8)
        
        # Current status
        self.current_status: Optional[BatteryStatus] = None
        
        # Historical data for trend analysis
        self.voltage_history = deque(maxlen=100)  # 10 seconds at 10Hz
        self.current_history = deque(maxlen=100)
        
        # Callbacks for alerts
        self.warning_callbacks: List[Callable] = []
        self.critical_callbacks: List[Callable] = []
        
        # Monitoring thread
        self.monitoring = False
        self.monitor_thread = None
        
        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ROS communication
        battery_topic = f'/{namespace}/mavros/battery' if namespace else '/mavros/battery'
        health_topic = f'/{namespace}/tee/health/battery' if namespace else '/tee/health/battery'
        
        self.battery_sub = self.node.create_subscription(
            BatteryState,
            battery_topic,
            self._battery_callback,
            qos_profile
        )
        
        self.health_pub = self.node.create_publisher(
            HealthReport,
            health_topic,
            10
        )
        
        self.node.get_logger().info("Battery monitor initialized")
    
    def start_monitoring(self):
        """Start continuous monitoring in background thread"""
        if self.monitoring:
            self.node.get_logger().warn("Battery monitoring already running")
            return
        
        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True
        )
        self.monitor_thread.start()
        self.node.get_logger().info("Battery monitoring started")
    
    def stop_monitoring(self):
        """Stop monitoring thread"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        self.node.get_logger().info("Battery monitoring stopped")
    
    def _battery_callback(self, msg: BatteryState):
        """
        Process incoming battery state messages.
        
        Args:
            msg: BatteryState message from MAVROS
        """
        # Calculate capacity in Wh
        capacity_wh = msg.voltage * (msg.percentage * self.battery_capacity_mah / 1000.0)
        
        # Extract cell voltages if available
        cell_voltages = []
        if hasattr(msg, 'cell_voltage') and msg.cell_voltage:
            cell_voltages = [v for v in msg.cell_voltage if v > 0]
        
        # Create status object
        self.current_status = BatteryStatus(
            voltage=msg.voltage,
            current=msg.current,
            percentage=msg.percentage,
            capacity_wh=capacity_wh,
            temperature=msg.temperature if hasattr(msg, 'temperature') else 25.0,
            cell_voltages=cell_voltages,
            health_state='unknown',
            timestamp=self.node.get_clock().now().nanoseconds / 1e9
        )
        
        # Update history
        self.voltage_history.append(msg.voltage)
        self.current_history.append(msg.current)
    
    def _monitoring_loop(self):
        """Main monitoring loop running in background thread"""
        import time
        
        while self.monitoring:
            if self.current_status is None:
                time.sleep(0.1)
                continue
            
            # Perform health assessment
            health_state = self._assess_battery_health()
            self.current_status.health_state = health_state
            
            # Publish health report
            self._publish_health_report()
            
            # Trigger callbacks based on health state
            if health_state == 'critical':
                self._trigger_critical_callbacks()
            elif health_state == 'warning':
                self._trigger_warning_callbacks()
            
            time.sleep(0.1)  # 10Hz
    
    def _assess_battery_health(self) -> str:
        """
        Perform comprehensive battery health assessment.
        
        Returns:
            Health state string: 'good', 'warning', 'critical', 'unknown'
        """
        if not self.current_status:
            return 'unknown'
        
        status = self.current_status
        
        # Level 1: Critical percentage threshold
        if status.percentage < self.critical_battery_pct:
            self.node.get_logger().error(f"CRITICAL: Battery at {status.percentage*100:.1f}%")
            return 'critical'
        
        # Level 2: Warning percentage threshold
        if status.percentage < self.min_battery_pct:
            self.node.get_logger().warn(f"WARNING: Battery at {status.percentage*100:.1f}%")
            return 'warning'
        
        # Level 3: Voltage sag detection
        if len(self.voltage_history) >= 20:
            voltage_trend = self._calculate_trend(self.voltage_history)
            if voltage_trend < -0.1:  # Rapid voltage drop (>0.1V/sec)
                self.node.get_logger().warn(f"WARNING: Rapid voltage drop: {voltage_trend:.3f}V/s")
                return 'warning'
        
        # Level 4: Cell imbalance detection
        if len(status.cell_voltages) > 1:
            cell_imbalance = max(status.cell_voltages) - min(status.cell_voltages)
            if cell_imbalance > 0.3:  # 300mV imbalance
                self.node.get_logger().warn(f"WARNING: Cell imbalance {cell_imbalance:.3f}V")
                return 'warning'
            elif cell_imbalance > 0.2:  # 200mV imbalance
                self.node.get_logger().debug(f"Cell imbalance detected: {cell_imbalance:.3f}V")
        
        # Level 5: Temperature check
        if status.temperature < 0 or status.temperature > 55:  # °C
            self.node.get_logger().warn(
                f"WARNING: Battery temperature {status.temperature:.1f}°C out of range"
            )
            return 'warning'
        
        # Level 6: High current draw detection
        if abs(status.current) > 50.0:  # 50A threshold
            self.node.get_logger().debug(f"High current draw: {status.current:.1f}A")
        
        return 'good'
    
    def _calculate_trend(self, data: deque) -> float:
        """
        Calculate trend (rate of change) from recent data.
        
        Args:
            data: Deque of recent values
            
        Returns:
            Rate of change per second
        """
        if len(data) < 2:
            return 0.0
        
        # Simple linear regression
        x = np.arange(len(data))
        y = np.array(data)
        
        # Calculate slope (change per sample)
        slope = np.polyfit(x, y, 1)[0]
        
        # Convert to per-second (assuming 10Hz sampling)
        slope_per_sec = slope * 10.0
        
        return slope_per_sec
    
    def _publish_health_report(self):
        """Publish battery health report"""
        if not self.current_status:
            return
        
        report = HealthReport()
        report.component = 'battery'
        report.state = self.current_status.health_state
        report.value = self.current_status.percentage
        report.message = (
            f"Battery: {self.current_status.percentage*100:.1f}%, "
            f"{self.current_status.voltage:.2f}V, "
            f"{self.current_status.current:.2f}A, "
            f"{self.current_status.temperature:.1f}°C"
        )
        report.timestamp = self.node.get_clock().now().to_msg()
        
        self.health_pub.publish(report)
    
    def register_warning_callback(self, callback: Callable):
        """Register callback for warning state"""
        self.warning_callbacks.append(callback)
    
    def register_critical_callback(self, callback: Callable):
        """Register callback for critical state"""
        self.critical_callbacks.append(callback)
    
    def _trigger_warning_callbacks(self):
        """Execute all warning callbacks"""
        for callback in self.warning_callbacks:
            try:
                callback(self.current_status)
            except Exception as e:
                self.node.get_logger().error(f"Warning callback failed: {e}")
    
    def _trigger_critical_callbacks(self):
        """Execute all critical callbacks"""
        for callback in self.critical_callbacks:
            try:
                callback(self.current_status)
            except Exception as e:
                self.node.get_logger().error(f"Critical callback failed: {e}")
    
    def get_current_status(self) -> Optional[BatteryStatus]:
        """Get current battery status"""
        return self.current_status
    
    def get_remaining_flight_time(self, avg_power_draw: float = None) -> Optional[float]:
        """
        Estimate remaining flight time in seconds.
        
        Args:
            avg_power_draw: Average power draw in watts (uses current if None)
            
        Returns:
            Estimated remaining time in seconds, or None if cannot estimate
        """
        if not self.current_status:
            return None
        
        if avg_power_draw is None:
            # Use current power draw
            avg_power_draw = abs(self.current_status.voltage * self.current_status.current)
        
        if avg_power_draw <= 0:
            return None
        
        # Remaining capacity in Wh
        remaining_wh = self.current_status.capacity_wh
        
        # Time = Energy / Power
        remaining_time_hours = remaining_wh / avg_power_draw
        remaining_time_seconds = remaining_time_hours * 3600.0
        
        return remaining_time_seconds
