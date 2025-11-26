#!/usr/bin/env python3
"""
Performance Monitor for Multi-Drone MOOFS System.

Monitors and logs:
- CPU/Memory usage per node
- Message latency (command to execution)
- Topic publish rates
- Connection status per drone
- System-wide metrics

Metrics are:
- Published to ROS topics for real-time monitoring
- Logged to CSV files for post-analysis
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

import psutil
import os
import json
import csv
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict, field
from collections import deque
import threading
import time

from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


@dataclass
class DroneMetrics:
    """Metrics for a single drone."""
    drone_id: int
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    last_heartbeat: float = 0.0
    heartbeat_latency_ms: float = 0.0
    position_update_rate_hz: float = 0.0
    command_latency_ms: float = 0.0
    primitives_executed: int = 0
    primitives_failed: int = 0
    last_position_x: float = 0.0
    last_position_y: float = 0.0
    last_position_z: float = 0.0


@dataclass
class SystemMetrics:
    """System-wide metrics."""
    timestamp: str
    num_drones_connected: int = 0
    num_drones_armed: int = 0
    total_cpu_percent: float = 0.0
    total_memory_mb: float = 0.0
    total_memory_percent: float = 0.0
    ros_process_count: int = 0
    avg_heartbeat_latency_ms: float = 0.0
    max_heartbeat_latency_ms: float = 0.0
    avg_position_rate_hz: float = 0.0
    total_messages_per_sec: float = 0.0


class PerformanceMonitor(Node):
    """
    Centralized performance monitoring for multi-drone system.
    
    Subscribes to MAVROS topics for each drone and aggregates metrics.
    """
    
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Parameters
        self.declare_parameter('sample_rate', 1.0)
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('log_directory', '/tmp/moofs_metrics')
        self.declare_parameter('max_drones', 10)
        self.declare_parameter('summary_interval', 5.0)
        
        self.sample_rate = self.get_parameter('sample_rate').value
        self.log_to_file = self.get_parameter('log_to_file').value
        self.log_directory = self.get_parameter('log_directory').value
        self.max_drones = self.get_parameter('max_drones').value
        self.summary_interval = self.get_parameter('summary_interval').value
        
        # Create log directory
        if self.log_to_file:
            os.makedirs(self.log_directory, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.metrics_file = os.path.join(
                self.log_directory, 
                f'system_metrics_{timestamp}.csv'
            )
            self.drone_metrics_file = os.path.join(
                self.log_directory,
                f'drone_metrics_{timestamp}.csv'
            )
            self._init_csv_files()
            self.get_logger().info(f'Logging metrics to: {self.log_directory}')
        
        # Drone metrics storage
        self.drone_metrics: Dict[int, DroneMetrics] = {}
        self.message_timestamps: Dict[str, deque] = {}
        self.lock = threading.Lock()
        
        # Track startup time
        self.start_time = datetime.now()
        self.samples_collected = 0
        
        # QoS for sensor data (best effort for high-rate topics)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to each drone's topics
        self.state_subs = {}
        self.pose_subs = {}
        
        for drone_id in range(self.max_drones):
            self._setup_drone_subscriptions(drone_id, sensor_qos)
        
        # Publisher for aggregated metrics (JSON format)
        self.metrics_pub = self.create_publisher(
            String, '/monitoring/system_metrics', 10
        )
        
        self.drone_metrics_pub = self.create_publisher(
            String, '/monitoring/drone_metrics', 10
        )
        
        # Timer for periodic metrics collection
        self.create_timer(1.0 / self.sample_rate, self.collect_metrics)
        
        # Timer for periodic logging summary
        self.create_timer(self.summary_interval, self.log_metrics_summary)
        
        self.get_logger().info(
            f'Performance Monitor started - tracking up to {self.max_drones} drones @ {self.sample_rate}Hz'
        )
    
    def _init_csv_files(self):
        """Initialize CSV files with headers."""
        # System metrics CSV
        with open(self.metrics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'elapsed_seconds', 'num_drones_connected', 'num_drones_armed',
                'total_cpu_percent', 'total_memory_mb', 'total_memory_percent',
                'ros_process_count', 'avg_heartbeat_latency_ms', 'max_heartbeat_latency_ms',
                'avg_position_rate_hz', 'total_messages_per_sec'
            ])
        
        # Drone metrics CSV
        with open(self.drone_metrics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'elapsed_seconds', 'drone_id', 'connected', 'armed', 'mode',
                'heartbeat_latency_ms', 'position_update_rate_hz',
                'position_x', 'position_y', 'position_z',
                'primitives_executed', 'primitives_failed'
            ])
    
    def _setup_drone_subscriptions(self, drone_id: int, qos: QoSProfile):
        """Setup subscriptions for a single drone."""
        namespace = f'/drone_{drone_id}'
        
        # Initialize metrics
        self.drone_metrics[drone_id] = DroneMetrics(drone_id=drone_id)
        self.message_timestamps[f'pose_{drone_id}'] = deque(maxlen=100)
        self.message_timestamps[f'state_{drone_id}'] = deque(maxlen=100)
        
        # MAVROS State subscription
        self.state_subs[drone_id] = self.create_subscription(
            State,
            f'{namespace}/mavros/state',
            lambda msg, did=drone_id: self._state_callback(msg, did),
            10
        )
        
        # Local Position subscription
        self.pose_subs[drone_id] = self.create_subscription(
            PoseStamped,
            f'{namespace}/mavros/local_position/pose',
            lambda msg, did=drone_id: self._pose_callback(msg, did),
            qos
        )
    
    def _state_callback(self, msg: State, drone_id: int):
        """Handle MAVROS state updates."""
        now = self.get_clock().now().nanoseconds / 1e9
        
        with self.lock:
            metrics = self.drone_metrics[drone_id]
            
            # Track state message timestamps
            timestamps = self.message_timestamps[f'state_{drone_id}']
            timestamps.append(now)
            
            # Calculate heartbeat latency (time between state messages)
            if len(timestamps) >= 2:
                metrics.heartbeat_latency_ms = (timestamps[-1] - timestamps[-2]) * 1000
            
            metrics.last_heartbeat = now
            metrics.connected = msg.connected
            metrics.armed = msg.armed
            metrics.mode = msg.mode
    
    def _pose_callback(self, msg: PoseStamped, drone_id: int):
        """Handle position updates and calculate update rate."""
        now = self.get_clock().now().nanoseconds / 1e9
        
        with self.lock:
            timestamps = self.message_timestamps[f'pose_{drone_id}']
            timestamps.append(now)
            
            # Calculate update rate
            if len(timestamps) >= 2:
                time_span = timestamps[-1] - timestamps[0]
                if time_span > 0:
                    self.drone_metrics[drone_id].position_update_rate_hz = \
                        (len(timestamps) - 1) / time_span
            
            # Store position
            metrics = self.drone_metrics[drone_id]
            metrics.last_position_x = msg.pose.position.x
            metrics.last_position_y = msg.pose.position.y
            metrics.last_position_z = msg.pose.position.z
    
    def collect_metrics(self):
        """Collect and publish system-wide metrics."""
        try:
            self.samples_collected += 1
            elapsed = (datetime.now() - self.start_time).total_seconds()
            
            # Get system-wide resource usage
            cpu_percent = psutil.cpu_percent()
            memory = psutil.virtual_memory()
            
            # Count ROS-related processes
            ros_process_count = 0
            for proc in psutil.process_iter(['name']):
                try:
                    name = proc.info['name'].lower()
                    if 'ros' in name or 'python' in name or 'mavros' in name:
                        ros_process_count += 1
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            
            with self.lock:
                # Count connected and armed drones
                connected_drones = sum(
                    1 for m in self.drone_metrics.values() if m.connected
                )
                armed_drones = sum(
                    1 for m in self.drone_metrics.values() if m.armed
                )
                
                # Calculate latency statistics
                latencies = [
                    m.heartbeat_latency_ms 
                    for m in self.drone_metrics.values() 
                    if m.connected and m.heartbeat_latency_ms > 0
                ]
                avg_heartbeat = sum(latencies) / len(latencies) if latencies else 0
                max_heartbeat = max(latencies) if latencies else 0
                
                # Calculate average position rate
                rates = [
                    m.position_update_rate_hz 
                    for m in self.drone_metrics.values() 
                    if m.connected
                ]
                avg_rate = sum(rates) / len(rates) if rates else 0
                total_rate = sum(rates)
            
            # Create system metrics
            system_metrics = SystemMetrics(
                timestamp=datetime.now().isoformat(),
                num_drones_connected=connected_drones,
                num_drones_armed=armed_drones,
                total_cpu_percent=cpu_percent,
                total_memory_mb=memory.used / 1024 / 1024,
                total_memory_percent=memory.percent,
                ros_process_count=ros_process_count,
                avg_heartbeat_latency_ms=avg_heartbeat,
                max_heartbeat_latency_ms=max_heartbeat,
                avg_position_rate_hz=avg_rate,
                total_messages_per_sec=total_rate,
            )
            
            # Publish system metrics as JSON
            msg = String()
            msg.data = json.dumps(asdict(system_metrics))
            self.metrics_pub.publish(msg)
            
            # Publish per-drone metrics as JSON
            with self.lock:
                drone_data = {
                    drone_id: asdict(metrics) 
                    for drone_id, metrics in self.drone_metrics.items()
                    if metrics.connected
                }
            drone_msg = String()
            drone_msg.data = json.dumps(drone_data)
            self.drone_metrics_pub.publish(drone_msg)
            
            # Log to CSV files
            if self.log_to_file:
                self._write_system_metrics(system_metrics, elapsed)
                self._write_drone_metrics(elapsed)
            
        except Exception as e:
            self.get_logger().error(f'Error collecting metrics: {e}')
    
    def _write_system_metrics(self, metrics: SystemMetrics, elapsed: float):
        """Write system metrics to CSV."""
        try:
            with open(self.metrics_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    metrics.timestamp, f'{elapsed:.2f}',
                    metrics.num_drones_connected, metrics.num_drones_armed,
                    f'{metrics.total_cpu_percent:.1f}', 
                    f'{metrics.total_memory_mb:.0f}',
                    f'{metrics.total_memory_percent:.1f}',
                    metrics.ros_process_count,
                    f'{metrics.avg_heartbeat_latency_ms:.2f}',
                    f'{metrics.max_heartbeat_latency_ms:.2f}',
                    f'{metrics.avg_position_rate_hz:.1f}',
                    f'{metrics.total_messages_per_sec:.1f}'
                ])
        except Exception as e:
            self.get_logger().warn(f'Error writing system metrics: {e}')
    
    def _write_drone_metrics(self, elapsed: float):
        """Write per-drone metrics to CSV."""
        timestamp = datetime.now().isoformat()
        try:
            with self.lock:
                with open(self.drone_metrics_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    for drone_id, metrics in self.drone_metrics.items():
                        if metrics.connected:
                            writer.writerow([
                                timestamp, f'{elapsed:.2f}',
                                drone_id, metrics.connected, metrics.armed,
                                metrics.mode, 
                                f'{metrics.heartbeat_latency_ms:.2f}',
                                f'{metrics.position_update_rate_hz:.1f}',
                                f'{metrics.last_position_x:.2f}',
                                f'{metrics.last_position_y:.2f}',
                                f'{metrics.last_position_z:.2f}',
                                metrics.primitives_executed, 
                                metrics.primitives_failed
                            ])
        except Exception as e:
            self.get_logger().warn(f'Error writing drone metrics: {e}')
    
    def log_metrics_summary(self):
        """Log a summary of current metrics to console."""
        with self.lock:
            connected = [
                (did, m) for did, m in self.drone_metrics.items() if m.connected
            ]
        
        elapsed = (datetime.now() - self.start_time).total_seconds()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(
            f'PERFORMANCE SUMMARY (elapsed: {elapsed:.0f}s, samples: {self.samples_collected})'
        )
        self.get_logger().info(f'Connected drones: {len(connected)}/{self.max_drones}')
        
        if connected:
            armed_count = sum(1 for _, m in connected if m.armed)
            self.get_logger().info(f'Armed drones: {armed_count}')
            
            for drone_id, metrics in connected:
                self.get_logger().info(
                    f'  Drone {drone_id}: mode={metrics.mode:10s} armed={str(metrics.armed):5s} '
                    f'rate={metrics.position_update_rate_hz:5.1f}Hz '
                    f'lat={metrics.heartbeat_latency_ms:6.1f}ms '
                    f'pos=({metrics.last_position_x:.1f}, {metrics.last_position_y:.1f}, {metrics.last_position_z:.1f})'
                )
        
        # System stats
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory()
        self.get_logger().info(
            f'System: CPU={cpu:.1f}%, Memory={mem.percent:.1f}% ({mem.used/1024/1024:.0f}MB)'
        )
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Performance monitor shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
