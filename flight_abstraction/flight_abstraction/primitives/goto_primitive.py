#!/usr/bin/env python3
"""Goto waypoint primitive for drone navigation."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry

from .base_primitive import BasePrimitive, PrimitiveState


class GotoPrimitive(BasePrimitive):
    """
    Primitive for navigating to a waypoint.
    
    This primitive commands the drone to fly to a target position and
    monitors progress until the waypoint is reached.
    """
    
    def __init__(self, node: Node, drone_namespace: str):
        """
        Initialize the goto primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
        """
        super().__init__(node, drone_namespace)
        
        # Create publisher for setpoint
        setpoint_topic = f"{drone_namespace}/mavros/setpoint_position/local"
        self.setpoint_pub = node.create_publisher(
            PoseStamped,
            setpoint_topic,
            10
        )
        
        # QoS profile for MAVROS topics (reliable matches MAVROS defaults)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to state and local position
        state_topic = f"{drone_namespace}/mavros/state"
        self.state_sub = node.create_subscription(
            State,
            state_topic,
            self._state_callback,
            qos_profile
        )
        
        local_pos_topic = f"{drone_namespace}/mavros/local_position/pose"
        self.local_pos_sub = node.create_subscription(
            PoseStamped,
            local_pos_topic,
            self._local_pos_callback,
            qos_profile
        )
        
        # State tracking
        self.current_state: Optional[State] = None
        self.current_position = Point()
        self.target_position = Point()
        self.target_heading = 0.0
        self.max_speed = 2.0  # m/s
        self.acceptance_radius = 0.5  # meters
        self.command_sent_time = None
        self.timeout = 60.0  # seconds
        self.last_setpoint_time = 0.0
        self.setpoint_rate = 20.0  # Hz
        
        self.logger.info(f"GotoPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_position = msg.pose.position
        
    def _calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calculate Euclidean distance between two points."""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def _calculate_yaw_from_heading(self, heading_deg: float) -> float:
        """
        Convert heading in degrees to yaw in radians.
        
        Args:
            heading_deg: Heading in degrees (0=North, 90=East)
            
        Returns:
            float: Yaw in radians
        """
        return math.radians(heading_deg)
    
    def execute(self, target_position: Point, target_heading: float = 0.0, 
                max_speed: float = 2.0, acceptance_radius: float = 0.5,
                timeout: float = 60.0) -> bool:
        """
        Execute navigation to target waypoint.
        
        Args:
            target_position: Target position (x, y, z) in local frame (meters)
            target_heading: Target heading in degrees (0=North, 90=East)
            max_speed: Maximum speed in m/s
            acceptance_radius: Distance threshold for waypoint reached (meters)
            timeout: Maximum time to wait for completion (seconds)
            
        Returns:
            bool: True if command was accepted, False otherwise
        """
        self.reset()
        self.target_position = target_position
        self.target_heading = target_heading
        self.max_speed = max_speed
        self.acceptance_radius = acceptance_radius
        self.timeout = timeout
        
        # Validate parameters
        if max_speed <= 0.0 or max_speed > 10.0:
            self.set_error(f"Invalid max speed: {max_speed} (must be 0-10 m/s)")
            return False
        
        if acceptance_radius <= 0.0 or acceptance_radius > 5.0:
            self.set_error(f"Invalid acceptance radius: {acceptance_radius}")
            return False
        
        # Check if armed and in GUIDED mode
        if self.current_state is None or not self.current_state.armed:
            self.set_error("Drone must be armed before navigation")
            return False
        
        if self.current_state.mode != "GUIDED":
            self.logger.warning(f"Drone not in GUIDED mode (current: {self.current_state.mode})")
        
        # Calculate initial distance
        initial_distance = self._calculate_distance(self.current_position, self.target_position)
        
        self.logger.info(f"Starting goto waypoint: target=({target_position.x:.2f}, {target_position.y:.2f}, {target_position.z:.2f}), "
                        f"distance={initial_distance:.2f}m, max_speed={max_speed:.1f}m/s")
        
        self.state = PrimitiveState.EXECUTING
        self.command_sent_time = time.time()
        
        return True
    
    def update(self) -> PrimitiveState:
        """
        Update primitive state and check for completion.
        
        This method should be called at high frequency to continuously
        send setpoint commands to MAVROS.
        
        Returns:
            PrimitiveState: Current state of the primitive
        """
        if self.state != PrimitiveState.EXECUTING:
            return self.state
        
        current_time = time.time()
        
        # Check timeout
        if self.command_sent_time is not None:
            elapsed = current_time - self.command_sent_time
            if elapsed > self.timeout:
                distance_remaining = self._calculate_distance(self.current_position, self.target_position)
                self.set_error(f"Goto timeout after {elapsed:.1f}s (distance remaining: {distance_remaining:.2f}m)")
                return self.state
        
        # Calculate distance to target
        distance_remaining = self._calculate_distance(self.current_position, self.target_position)
        
        # Calculate progress (inverse of distance, capped at 95%)
        if self.command_sent_time is not None:
            initial_distance = self._calculate_distance(self.current_position, self.target_position)
            if initial_distance > 0:
                progress_ratio = 1.0 - (distance_remaining / max(initial_distance, 1.0))
                self.progress = min(95.0, max(0.0, progress_ratio * 100.0))
        
        # Check if waypoint reached
        if distance_remaining < self.acceptance_radius:
            self.logger.info(f"Waypoint reached: distance={distance_remaining:.2f}m < {self.acceptance_radius:.2f}m")
            self.set_success()
            return self.state
        
        # Send setpoint at regular intervals
        if (current_time - self.last_setpoint_time) >= (1.0 / self.setpoint_rate):
            self._send_setpoint()
            self.last_setpoint_time = current_time
            
            # Log progress periodically
            if int(elapsed) % 2 == 0:  # Every 2 seconds
                self.logger.debug(f"Goto progress: distance={distance_remaining:.2f}m, progress={self.progress:.1f}%")
        
        return self.state
    
    def _send_setpoint(self):
        """Send position setpoint to MAVROS."""
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set target position
        msg.pose.position.x = self.target_position.x
        msg.pose.position.y = self.target_position.y
        msg.pose.position.z = self.target_position.z
        
        # Set orientation (yaw from heading)
        yaw = self._calculate_yaw_from_heading(self.target_heading)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.setpoint_pub.publish(msg)
    
    def cancel(self) -> bool:
        """
        Cancel the goto operation.
        
        This will stop sending setpoints and transition to hover.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            distance_remaining = self._calculate_distance(self.current_position, self.target_position)
            self.logger.warning(f"Cancelling goto (distance remaining: {distance_remaining:.2f}m)")
            return super().cancel()
        return False
    
    def get_distance_remaining(self) -> float:
        """
        Get distance to target waypoint.
        
        Returns:
            float: Distance in meters
        """
        return self._calculate_distance(self.current_position, self.target_position)
    
    def get_estimated_time_remaining(self) -> float:
        """
        Get estimated time to reach waypoint.
        
        Returns:
            float: Estimated time in seconds
        """
        distance = self.get_distance_remaining()
        if self.max_speed > 0:
            return distance / self.max_speed
        return 0.0
