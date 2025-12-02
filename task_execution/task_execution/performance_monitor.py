#!/usr/bin/env python3
"""
Performance Monitor for Multi-Drone MOOFS System.

Monitors and logs:
- CPU/Memory usage per node
- Message latency (command to execution)
- Topic publish rates
- Connection status per drone
- System-wide metrics

FIXED: Proper latency calculation using message timestamps
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
from std_msgs.msg import String, Header


@dataclass
class DroneMetrics:
    """Metrics for a single drone."""
    drone_id: int
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    
    # Timing metrics
    last_state_time: float = 0.0
    last_pose_time: float = 0.0
    state_msg_latency_ms: float = 0.0  # Message timestamp vs receive time
    pose_msg_latency_ms: float = 0.0
    
    # Rate metrics
    state_rate_hz: float = 0.0
    pose_rate_hz: float = 0.0
    
    # Position
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    
    # Counters
    state_count: int = 0
    pose_count: int = 0
    primitives_executed: int = 0
    primitives_failed: int = 0


@dataclass 
class SystemMetrics:
    """System-wide metrics."""
    timestamp: str = ""
    elapsed_seconds: float = 0.0
    sample_count: int = 0
    
    # Drone counts
    num_drones_connected: int = 0
    num_drones_armed: int = 0
    
    # System resources
    total_cpu_percent: float = 0.0
    total_memory_mb: float = 0.0
    total_memory_percent: float = 0.0
    
    # Per-process resources
    ros_process_count: int = 0
    ros_cpu_percent: float = 0.0
    ros_memory_mb: float = 0.0
    
    # Latency stats (message timestamp to receive time)
    avg_state_latency_ms: float = 0.0
    max_state_latency_ms: float = 0.0
    min_state_latency_ms: float = 0.0
    
    # Rate stats
    avg_state_rate_hz: float = 0.0
    avg_pose_rate_hz: float = 0.0
    total_messages_per_sec: float = 0.0


class PerformanceMonitor(Node):
    """
    Centralized performance monitoring for multi-drone system.
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
        
        # Timing
        self.start_time = self.get_clock().now()
        self.sample_count = 0
        
        # Create log directory
        if self.log_to_file:
            os.makedirs(self.log_directory, exist_ok=True)
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.metrics_file = os.path.join(
                self.log_directory, 
                f'system_metrics_{timestamp_str}.csv'
            )
            self.drone_metrics_file = os.path.join(
                self.log_directory,
                f'drone_metrics_{timestamp_str}.csv'
            )
            self._init_csv_files()
            self.get_logger().info(f'Logging metrics to: {self.log_directory}')
        
        # Drone metrics storage
        self.drone_metrics: Dict[int, DroneMetrics] = {}
        self.state_timestamps: Dict[int, deque] = {}
        self.pose_timestamps: Dict[int, deque] = {}
        self.lock = threading.Lock()
        
        # QoS for sensor data
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
        
        # Publisher for aggregated metrics
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
                'timestamp', 'elapsed_s', 'sample', 'drones_connected', 'drones_armed',
                'cpu_percent', 'memory_mb', 'memory_percent',
                'ros_processes', 'ros_cpu_percent', 'ros_memory_mb',
                'avg_state_latency_ms', 'max_state_latency_ms', 'min_state_latency_ms',
                'avg_state_rate_hz', 'avg_pose_rate_hz', 'total_msg_rate'
            ])
        
        # Drone metrics CSV
        with open(self.drone_metrics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp', 'elapsed_s', 'drone_id', 'connected', 'armed', 'mode',
                'state_latency_ms', 'pose_latency_ms',
                'state_rate_hz', 'pose_rate_hz',
                'pos_x', 'pos_y', 'pos_z',
                'state_count', 'pose_count'
            ])
    
    def _setup_drone_subscriptions(self, drone_id: int, qos: QoSProfile):
        """Setup subscriptions for a single drone."""
        namespace = f'/drone_{drone_id}'
        
        # Initialize metrics
        self.drone_metrics[drone_id] = DroneMetrics(drone_id=drone_id)
        self.state_timestamps[drone_id] = deque(maxlen=100)
        self.pose_timestamps[drone_id] = deque(maxlen=100)
        
        # MAVROS State
        self.state_subs[drone_id] = self.create_subscription(
            State,
            f'{namespace}/mavros/state',
            lambda msg, did=drone_id: self._state_callback(msg, did),
            10
        )
        
        # Local Position
        self.pose_subs[drone_id] = self.create_subscription(
            PoseStamped,
            f'{namespace}/mavros/local_position/pose',
            lambda msg, did=drone_id: self._pose_callback(msg, did),
            qos
        )
    
    def _calculate_msg_latency(self, header: Header) -> float:
        """Calculate latency from message timestamp to now (in ms)."""
        if header.stamp.sec == 0 and header.stamp.nanosec == 0:
            return -1.0  # No valid timestamp
        
        now = self.get_clock().now()
        msg_time = Time(seconds=header.stamp.sec, nanoseconds=header.stamp.nanosec)
        
        # Handle clock differences (SITL may have different time)
        latency_ns = now.nanoseconds - msg_time.nanoseconds
        latency_ms = latency_ns / 1e6
        
        # If negative or huge, clocks are not synced - use receive time delta
        if latency_ms < 0 or latency_ms > 10000:
            return -1.0
        
        return latency_ms
    
    def _calculate_rate(self, timestamps: deque) -> float:
        """Calculate message rate from timestamp history."""
        if len(timestamps) < 2:
            return 0.0
        
        time_span = timestamps[-1] - timestamps[0]
        if time_span <= 0:
            return 0.0
        
        return (len(timestamps) - 1) / time_span
    
    def _state_callback(self, msg: State, drone_id: int):
        """Handle MAVROS state updates."""
        now = self.get_clock().now().nanoseconds / 1e9
        
        with self.lock:
            metrics = self.drone_metrics[drone_id]
            
            # Update state info
            metrics.connected = msg.connected
            metrics.armed = msg.armed
            metrics.mode = msg.mode
            
            # Track timing
            self.state_timestamps[drone_id].append(now)
            metrics.state_count += 1
            
            # Calculate rate
            metrics.state_rate_hz = self._calculate_rate(self.state_timestamps[drone_id])
            
            # Calculate latency (time between state messages)
            if metrics.last_state_time > 0:
                delta_ms = (now - metrics.last_state_time) * 1000
                metrics.state_msg_latency_ms = delta_ms
            
            metrics.last_state_time = now
    
    def _pose_callback(self, msg: PoseStamped, drone_id: int):
        """Handle position updates."""
        now = self.get_clock().now().nanoseconds / 1e9
        
        with self.lock:
            metrics = self.drone_metrics[drone_id]
            
            # Track timing
            self.pose_timestamps[drone_id].append(now)
            metrics.pose_count += 1
            
            # Calculate rate
            metrics.pose_rate_hz = self._calculate_rate(self.pose_timestamps[drone_id])
            
            # Calculate latency from message header
            latency = self._calculate_msg_latency(msg.header)
            if latency >= 0:
                metrics.pose_msg_latency_ms = latency
            
            # Update position
            metrics.position_x = msg.pose.position.x
            metrics.position_y = msg.pose.position.y
            metrics.position_z = msg.pose.position.z
            
            metrics.last_pose_time = now
    
    def collect_metrics(self):
        """Collect and publish system-wide metrics."""
        try:
            self.sample_count += 1
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            
            # Get process metrics
            ros_processes = 0
            ros_cpu = 0.0
            ros_memory = 0.0
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'cpu_percent', 'memory_info']):
                try:
                    cmdline = ' '.join(proc.info['cmdline'] or []).lower()
                    name = (proc.info['name'] or '').lower()
                    
                    # Check if it's a ROS-related process
                    if any(x in cmdline or x in name for x in ['ros', 'mavros', 'fal_node', 'tee_node', 'python3']):
                        ros_processes += 1
                        ros_cpu += proc.info['cpu_percent'] or 0
                        if proc.info['memory_info']:
                            ros_memory += proc.info['memory_info'].rss / 1024 / 1024
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass
            
            with self.lock:
                # Count connected/armed drones
                connected_drones = sum(1 for m in self.drone_metrics.values() if m.connected)
                armed_drones = sum(1 for m in self.drone_metrics.values() if m.armed)
                
                # Calculate latency statistics (only for connected drones)
                latencies = [
                    m.state_msg_latency_ms 
                    for m in self.drone_metrics.values() 
                    if m.connected and m.state_msg_latency_ms > 0
                ]
                
                avg_latency = sum(latencies) / len(latencies) if latencies else 0
                max_latency = max(latencies) if latencies else 0
                min_latency = min(latencies) if latencies else 0
                
                # Calculate rate statistics
                state_rates = [m.state_rate_hz for m in self.drone_metrics.values() if m.connected]
                pose_rates = [m.pose_rate_hz for m in self.drone_metrics.values() if m.connected]
                
                avg_state_rate = sum(state_rates) / len(state_rates) if state_rates else 0
                avg_pose_rate = sum(pose_rates) / len(pose_rates) if pose_rates else 0
                total_msg_rate = sum(state_rates) + sum(pose_rates)
            
            # Create system metrics
            mem = psutil.virtual_memory()
            system_metrics = SystemMetrics(
                timestamp=datetime.now().isoformat(),
                elapsed_seconds=elapsed,
                sample_count=self.sample_count,
                num_drones_connected=connected_drones,
                num_drones_armed=armed_drones,
                total_cpu_percent=psutil.cpu_percent(),
                total_memory_mb=mem.used / 1024 / 1024,
                total_memory_percent=mem.percent,
                ros_process_count=ros_processes,
                ros_cpu_percent=ros_cpu,
                ros_memory_mb=ros_memory,
                avg_state_latency_ms=avg_latency,
                max_state_latency_ms=max_latency,
                min_state_latency_ms=min_latency,
                avg_state_rate_hz=avg_state_rate,
                avg_pose_rate_hz=avg_pose_rate,
                total_messages_per_sec=total_msg_rate,
            )
            
            # Publish metrics as JSON
            msg = String()
            msg.data = json.dumps(asdict(system_metrics))
            self.metrics_pub.publish(msg)
            
            # Log to file
            if self.log_to_file:
                self._write_system_metrics(system_metrics)
                self._write_drone_metrics(elapsed)
            
        except Exception as e:
            self.get_logger().error(f'Error collecting metrics: {e}')
    
    def _write_system_metrics(self, metrics: SystemMetrics):
        """Write system metrics to CSV."""
        try:
            with open(self.metrics_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    metrics.timestamp, f'{metrics.elapsed_seconds:.1f}', metrics.sample_count,
                    metrics.num_drones_connected, metrics.num_drones_armed,
                    f'{metrics.total_cpu_percent:.1f}', f'{metrics.total_memory_mb:.0f}', 
                    f'{metrics.total_memory_percent:.1f}',
                    metrics.ros_process_count, f'{metrics.ros_cpu_percent:.1f}', 
                    f'{metrics.ros_memory_mb:.0f}',
                    f'{metrics.avg_state_latency_ms:.1f}', f'{metrics.max_state_latency_ms:.1f}',
                    f'{metrics.min_state_latency_ms:.1f}',
                    f'{metrics.avg_state_rate_hz:.2f}', f'{metrics.avg_pose_rate_hz:.2f}',
                    f'{metrics.total_messages_per_sec:.1f}'
                ])
        except Exception as e:
            self.get_logger().error(f'Error writing system metrics: {e}')
    
    def _write_drone_metrics(self, elapsed: float):
        """Write per-drone metrics to CSV."""
        timestamp = datetime.now().isoformat()
        try:
            with self.lock:
                with open(self.drone_metrics_file, 'a', newline='') as f:
                    writer = csv.writer(f)
                    for drone_id, m in self.drone_metrics.items():
                        if m.connected:
                            writer.writerow([
                                timestamp, f'{elapsed:.1f}', drone_id,
                                m.connected, m.armed, m.mode,
                                f'{m.state_msg_latency_ms:.1f}', f'{m.pose_msg_latency_ms:.1f}',
                                f'{m.state_rate_hz:.2f}', f'{m.pose_rate_hz:.2f}',
                                f'{m.position_x:.2f}', f'{m.position_y:.2f}', f'{m.position_z:.2f}',
                                m.state_count, m.pose_count
                            ])
        except Exception as e:
            self.get_logger().error(f'Error writing drone metrics: {e}')
    
    def log_metrics_summary(self):
        """Log a summary of current metrics to console."""
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        with self.lock:
            connected = [(did, m) for did, m in self.drone_metrics.items() if m.connected]
            armed = sum(1 for _, m in connected if m.armed)
        
        # Header
        self.get_logger().info('=' * 70)
        self.get_logger().info(
            f'PERFORMANCE SUMMARY (elapsed: {elapsed:.0f}s, samples: {self.sample_count})'
        )
        self.get_logger().info(f'Connected drones: {len(connected)}/{self.max_drones}')
        self.get_logger().info(f'Armed drones: {armed}')
        
        # Per-drone details
        if connected:
            for drone_id, m in sorted(connected, key=lambda x: x[0]):
                self.get_logger().info(
                    f'  Drone {drone_id}: mode={m.mode:<10} armed={str(m.armed):<5} '
                    f'state_rate={m.state_rate_hz:5.1f}Hz '
                    f'pose_rate={m.pose_rate_hz:5.1f}Hz '
                    f'msgs={m.state_count + m.pose_count}'
                )
        
        # System stats
        cpu = psutil.cpu_percent()
        mem = psutil.virtual_memory()
        self.get_logger().info(
            f'System: CPU={cpu:.1f}%, Memory={mem.percent:.1f}% ({mem.used/1024/1024:.0f}MB)'
        )
        
        # Latency stats
        with self.lock:
            latencies = [m.state_msg_latency_ms for m in self.drone_metrics.values() 
                        if m.connected and m.state_msg_latency_ms > 0]
        
        if latencies:
            self.get_logger().info(
                f'State msg interval: avg={sum(latencies)/len(latencies):.0f}ms, '
                f'min={min(latencies):.0f}ms, max={max(latencies):.0f}ms'
            )
        
        self.get_logger().info('=' * 70)


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down performance monitor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
