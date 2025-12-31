#!/usr/bin/env python3
"""Hover primitive for holding position at current location."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point

from .base_primitive import BasePrimitive, PrimitiveState


class HoverPrimitive(BasePrimitive):
    """
    Primitive for hovering at current position.
    
    This primitive commands the drone to hold its current position by
    continuously publishing position setpoints. It maintains position
    until cancelled or a duration expires.
    """
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the hover primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create publisher for setpoint
        setpoint_topic = f"{drone_namespace}/mavros/setpoint_position/local"
        self.setpoint_pub = node.create_publisher(
            PoseStamped,
            setpoint_topic,
            10
        )
        
        # QoS profiles
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to state and local position
        state_topic = f"{drone_namespace}/mavros/state"
        self.state_sub = node.create_subscription(
            State,
            state_topic,
            self._state_callback,
            state_qos,
            callback_group=callback_group
        )
        
        local_pos_topic = f"{drone_namespace}/mavros/local_position/pose"
        self.local_pos_sub = node.create_subscription(
            PoseStamped,
            local_pos_topic,
            self._local_pos_callback,
            pose_qos,
            callback_group=callback_group
        )
        
        # State tracking
        self.current_state: Optional[State] = None
        self.current_pose: Optional[PoseStamped] = None
        self.hover_position = Point()
        self.hover_orientation = None
        self.hold_duration = 0.0  # 0 = indefinite
        self.position_tolerance = 1.0  # meters
        self.command_sent_time = None
        self.last_setpoint_time = 0.0
        self.setpoint_rate = 20.0  # Hz
        
        self.logger.info(f"HoverPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_pose = msg
        
    def _calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calculate 3D Euclidean distance between two points."""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def execute(self, duration: float = 0.0, position_tolerance: float = 1.0) -> bool:
        """
        Execute hover at current position.
        
        Args:
            duration: How long to hover in seconds (0 = indefinite until cancelled)
            position_tolerance: Maximum allowed drift from hover point (meters)
            
        Returns:
            bool: True if hover started successfully, False otherwise
        """
        self.reset()
        self.hold_duration = duration
        self.position_tolerance = position_tolerance
        
        # Check if we have position information
        if self.current_pose is None:
            self.set_error("No position information available")
            return False
        
        # Check if armed
        if self.current_state is None or not self.current_state.armed:
            self.set_error("Drone must be armed to hover")
            return False
        
        # Capture current position as hover point
        self.hover_position.x = self.current_pose.pose.position.x
        self.hover_position.y = self.current_pose.pose.position.y
        self.hover_position.z = self.current_pose.pose.position.z
        self.hover_orientation = self.current_pose.pose.orientation
        
        self.logger.info(f"Starting hover at position: ({self.hover_position.x:.2f}, "
                        f"{self.hover_position.y:.2f}, {self.hover_position.z:.2f})")
        
        if duration > 0:
            self.logger.info(f"Hover duration: {duration:.1f}s")
        else:
            self.logger.info("Hover duration: indefinite (until cancelled)")
        
        self.state = PrimitiveState.EXECUTING
        self.command_sent_time = time.time()
        
        return True
    
    def update(self) -> PrimitiveState:
        """
        Update primitive state and publish hover setpoint.
        
        Returns:
            PrimitiveState: Current state of the primitive
        """
        if self.state != PrimitiveState.EXECUTING:
            return self.state
        
        current_time = time.time()
        
        # Check if duration expired (only if duration > 0)
        if self.hold_duration > 0 and self.command_sent_time is not None:
            elapsed = current_time - self.command_sent_time
            if elapsed >= self.hold_duration:
                self.logger.info(f"Hover duration completed: {elapsed:.1f}s")
                self.set_success()
                return self.state
            
            # Update progress
            self.progress = min(95.0, (elapsed / self.hold_duration) * 100.0)
        
        # Check position drift
        if self.current_pose is not None:
            current_pos = self.current_pose.pose.position
            drift = self._calculate_distance(current_pos, self.hover_position)
            
            if drift > self.position_tolerance:
                self.logger.warning(f"Position drift detected: {drift:.2f}m > {self.position_tolerance:.2f}m")
        
        # Publish hover setpoint at regular intervals
        if (current_time - self.last_setpoint_time) >= (1.0 / self.setpoint_rate):
            self._send_setpoint()
            self.last_setpoint_time = current_time
        
        return self.state
    
    def _send_setpoint(self):
        """Send position setpoint to maintain hover."""
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set hover position
        msg.pose.position.x = self.hover_position.x
        msg.pose.position.y = self.hover_position.y
        msg.pose.position.z = self.hover_position.z
        
        # Maintain original orientation
        if self.hover_orientation is not None:
            msg.pose.orientation = self.hover_orientation
        else:
            msg.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(msg)
    
    def get_drift(self) -> float:
        """
        Get current position drift from hover point.
        
        Returns:
            float: Distance from hover point in meters
        """
        if self.current_pose is None:
            return 0.0
        return self._calculate_distance(self.current_pose.pose.position, self.hover_position)
    
    def cancel(self) -> bool:
        """
        Cancel the hover operation.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            self.logger.info("Hover cancelled")
            return super().cancel()
        return False
