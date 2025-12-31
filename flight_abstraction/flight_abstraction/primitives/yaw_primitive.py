#!/usr/bin/env python3
"""Yaw primitive for rotating drone to target heading."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from .base_primitive import BasePrimitive, PrimitiveState


class YawPrimitive(BasePrimitive):
    """
    Primitive for rotating the drone to a target heading.
    
    This primitive commands the drone to rotate in place to face a 
    specified heading while maintaining its current position.
    """
    
    # PositionTarget type masks
    IGNORE_VX = 8
    IGNORE_VY = 16
    IGNORE_VZ = 32
    IGNORE_AFX = 64
    IGNORE_AFY = 128
    IGNORE_AFZ = 256
    IGNORE_YAW_RATE = 2048
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the yaw primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create publisher for raw setpoint
        setpoint_raw_topic = f"{drone_namespace}/mavros/setpoint_raw/local"
        self.setpoint_raw_pub = node.create_publisher(
            PositionTarget,
            setpoint_raw_topic,
            10
        )
        
        # Create publisher for position setpoint
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
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.initial_yaw = 0.0
        self.yaw_rate = 0.5  # rad/s
        self.yaw_tolerance = 0.05  # radians (~3 degrees)
        self.command_sent_time = None
        self.timeout = 30.0  # seconds
        self.last_setpoint_time = 0.0
        self.setpoint_rate = 20.0  # Hz
        
        self.logger.info(f"YawPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_pose = msg
        
        # Extract yaw from quaternion
        q = msg.pose.orientation
        try:
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.current_yaw = yaw
        except Exception:
            pass
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _angle_difference(self, target: float, current: float) -> float:
        """Calculate shortest angular difference."""
        diff = target - current
        return self._normalize_angle(diff)
    
    def execute(self, target_yaw: float = None, target_heading_deg: float = None,
                yaw_rate: float = 0.5, tolerance_deg: float = 3.0,
                timeout: float = 30.0) -> bool:
        """
        Execute yaw rotation to target heading.
        
        Args:
            target_yaw: Target yaw in radians (takes precedence)
            target_heading_deg: Target heading in degrees (0=North, 90=East)
            yaw_rate: Maximum yaw rate in rad/s
            tolerance_deg: Acceptance tolerance in degrees
            timeout: Maximum time to complete rotation (seconds)
            
        Returns:
            bool: True if command started successfully, False otherwise
        """
        self.reset()
        self.yaw_rate = yaw_rate
        self.yaw_tolerance = math.radians(tolerance_deg)
        self.timeout = timeout
        
        # Determine target yaw
        if target_yaw is not None:
            self.target_yaw = self._normalize_angle(target_yaw)
        elif target_heading_deg is not None:
            # Convert heading (0=North, CW positive) to yaw (0=East, CCW positive)
            self.target_yaw = self._normalize_angle(math.radians(90.0 - target_heading_deg))
        else:
            self.set_error("Must specify either target_yaw or target_heading_deg")
            return False
        
        # Check if we have position information
        if self.current_pose is None:
            self.set_error("No position information available")
            return False
        
        # Check if armed
        if self.current_state is None or not self.current_state.armed:
            self.set_error("Drone must be armed for yaw control")
            return False
        
        self.initial_yaw = self.current_yaw
        
        # Check if already at target yaw
        yaw_error = abs(self._angle_difference(self.target_yaw, self.current_yaw))
        if yaw_error < self.yaw_tolerance:
            self.logger.info(f"Already at target yaw: {math.degrees(self.current_yaw):.1f}°")
            self.set_success()
            return True
        
        self.logger.info(f"Starting yaw rotation: {math.degrees(self.current_yaw):.1f}° -> "
                        f"{math.degrees(self.target_yaw):.1f}° (delta: {math.degrees(yaw_error):.1f}°)")
        
        self.state = PrimitiveState.EXECUTING
        self.command_sent_time = time.time()
        
        return True
    
    def update(self) -> PrimitiveState:
        """
        Update primitive state and check for completion.
        
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
                yaw_error = self._angle_difference(self.target_yaw, self.current_yaw)
                self.set_error(f"Yaw timeout after {elapsed:.1f}s (error: {math.degrees(yaw_error):.1f}°)")
                return self.state
        
        # Calculate yaw error
        yaw_error = self._angle_difference(self.target_yaw, self.current_yaw)
        abs_error = abs(yaw_error)
        
        # Calculate progress
        total_rotation = abs(self._angle_difference(self.target_yaw, self.initial_yaw))
        if total_rotation > 0.01:
            completed = total_rotation - abs_error
            self.progress = min(95.0, max(0.0, (completed / total_rotation) * 100.0))
        
        # Check if target reached
        if abs_error < self.yaw_tolerance:
            self.logger.info(f"Target yaw reached: {math.degrees(self.current_yaw):.1f}°")
            self.set_success()
            return self.state
        
        # Publish setpoint at regular intervals
        if (current_time - self.last_setpoint_time) >= (1.0 / self.setpoint_rate):
            self._send_setpoint()
            self.last_setpoint_time = current_time
        
        return self.state
    
    def _send_setpoint(self):
        """Send position setpoint with target yaw."""
        if self.current_pose is None:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Maintain current position
        msg.pose.position = self.current_pose.pose.position
        
        # Set target yaw orientation
        q = quaternion_from_euler(0, 0, self.target_yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.setpoint_pub.publish(msg)
    
    def get_yaw_error(self) -> float:
        """
        Get current yaw error in radians.
        
        Returns:
            float: Yaw error in radians
        """
        return self._angle_difference(self.target_yaw, self.current_yaw)
    
    def get_yaw_error_degrees(self) -> float:
        """
        Get current yaw error in degrees.
        
        Returns:
            float: Yaw error in degrees
        """
        return math.degrees(self.get_yaw_error())
    
    def cancel(self) -> bool:
        """
        Cancel the yaw operation.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            self.logger.info(f"Yaw cancelled at {math.degrees(self.current_yaw):.1f}°")
            return super().cancel()
        return False
