#!/usr/bin/env python3
"""Velocity primitive for commanding drone velocity."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.msg import State, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3

from .base_primitive import BasePrimitive, PrimitiveState


class VelocityPrimitive(BasePrimitive):
    """
    Primitive for commanding drone velocity.
    
    This primitive commands the drone to fly at a specified velocity
    (vx, vy, vz) in the local frame. Useful for smooth curved paths,
    tracking, and dynamic maneuvers.
    """
    
    # PositionTarget type masks for velocity control
    IGNORE_PX = 1
    IGNORE_PY = 2
    IGNORE_PZ = 4
    IGNORE_VX = 8
    IGNORE_VY = 16
    IGNORE_VZ = 32
    IGNORE_AFX = 64
    IGNORE_AFY = 128
    IGNORE_AFZ = 256
    IGNORE_YAW = 1024
    IGNORE_YAW_RATE = 2048
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the velocity primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create publisher for raw setpoint (supports velocity control)
        setpoint_raw_topic = f"{drone_namespace}/mavros/setpoint_raw/local"
        self.setpoint_raw_pub = node.create_publisher(
            PositionTarget,
            setpoint_raw_topic,
            10
        )
        
        # Alternative: velocity setpoint topic
        setpoint_vel_topic = f"{drone_namespace}/mavros/setpoint_velocity/cmd_vel"
        self.setpoint_vel_pub = node.create_publisher(
            TwistStamped,
            setpoint_vel_topic,
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
        self.target_velocity = Vector3()
        self.target_yaw_rate = 0.0
        self.target_yaw = None  # None = use yaw_rate, value = hold specific yaw
        self.duration = 0.0
        self.max_speed = 5.0  # m/s safety limit
        self.command_sent_time = None
        self.last_setpoint_time = 0.0
        self.setpoint_rate = 20.0  # Hz
        self.use_raw_setpoint = True  # Use PositionTarget for more control
        
        self.logger.info(f"VelocityPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_pose = msg
    
    def _clamp_velocity(self, velocity: Vector3) -> Vector3:
        """Clamp velocity components to max speed."""
        clamped = Vector3()
        clamped.x = max(-self.max_speed, min(self.max_speed, velocity.x))
        clamped.y = max(-self.max_speed, min(self.max_speed, velocity.y))
        clamped.z = max(-self.max_speed, min(self.max_speed, velocity.z))
        return clamped
    
    def execute(self, velocity: Vector3, duration: float = 0.0, 
                yaw: Optional[float] = None, yaw_rate: float = 0.0,
                max_speed: float = 5.0) -> bool:
        """
        Execute velocity command.
        
        Args:
            velocity: Target velocity (vx, vy, vz) in m/s (local frame)
            duration: How long to maintain velocity (0 = indefinite)
            yaw: Target yaw angle in radians (None = use yaw_rate)
            yaw_rate: Yaw rate in rad/s (used if yaw is None)
            max_speed: Maximum speed limit in m/s
            
        Returns:
            bool: True if command started successfully, False otherwise
        """
        self.reset()
        self.duration = duration
        self.max_speed = max_speed
        self.target_yaw = yaw
        self.target_yaw_rate = yaw_rate
        
        # Validate parameters
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        if speed > max_speed:
            self.logger.warning(f"Velocity magnitude {speed:.2f} exceeds max {max_speed:.2f}, will be clamped")
        
        # Clamp velocity
        self.target_velocity = self._clamp_velocity(velocity)
        
        # Check if armed
        if self.current_state is None or not self.current_state.armed:
            self.set_error("Drone must be armed for velocity control")
            return False
        
        # Check if in GUIDED mode
        if self.current_state.mode != "GUIDED":
            self.logger.warning(f"Drone not in GUIDED mode (current: {self.current_state.mode})")
        
        self.logger.info(f"Starting velocity command: ({self.target_velocity.x:.2f}, "
                        f"{self.target_velocity.y:.2f}, {self.target_velocity.z:.2f}) m/s")
        
        if duration > 0:
            self.logger.info(f"Duration: {duration:.1f}s")
        else:
            self.logger.info("Duration: indefinite (until cancelled)")
        
        self.state = PrimitiveState.EXECUTING
        self.command_sent_time = time.time()
        
        return True
    
    def update(self) -> PrimitiveState:
        """
        Update primitive state and publish velocity setpoint.
        
        Returns:
            PrimitiveState: Current state of the primitive
        """
        if self.state != PrimitiveState.EXECUTING:
            return self.state
        
        current_time = time.time()
        
        # Check if duration expired
        if self.duration > 0 and self.command_sent_time is not None:
            elapsed = current_time - self.command_sent_time
            if elapsed >= self.duration:
                self.logger.info(f"Velocity command completed: {elapsed:.1f}s")
                self.set_success()
                return self.state
            
            # Update progress
            self.progress = min(95.0, (elapsed / self.duration) * 100.0)
        
        # Publish velocity setpoint at regular intervals
        if (current_time - self.last_setpoint_time) >= (1.0 / self.setpoint_rate):
            if self.use_raw_setpoint:
                self._send_raw_setpoint()
            else:
                self._send_velocity_setpoint()
            self.last_setpoint_time = current_time
        
        return self.state
    
    def _send_raw_setpoint(self):
        """Send velocity command via PositionTarget (raw setpoint)."""
        msg = PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Ignore position and acceleration, use velocity
        msg.type_mask = (
            self.IGNORE_PX | self.IGNORE_PY | self.IGNORE_PZ |
            self.IGNORE_AFX | self.IGNORE_AFY | self.IGNORE_AFZ
        )
        
        # Set velocity
        msg.velocity.x = self.target_velocity.x
        msg.velocity.y = self.target_velocity.y
        msg.velocity.z = self.target_velocity.z
        
        # Set yaw or yaw_rate
        if self.target_yaw is not None:
            msg.type_mask |= self.IGNORE_YAW_RATE
            msg.yaw = self.target_yaw
        else:
            msg.type_mask |= self.IGNORE_YAW
            msg.yaw_rate = self.target_yaw_rate
        
        self.setpoint_raw_pub.publish(msg)
    
    def _send_velocity_setpoint(self):
        """Send velocity command via TwistStamped."""
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set linear velocity
        msg.twist.linear.x = self.target_velocity.x
        msg.twist.linear.y = self.target_velocity.y
        msg.twist.linear.z = self.target_velocity.z
        
        # Set angular velocity (yaw rate)
        msg.twist.angular.z = self.target_yaw_rate
        
        self.setpoint_vel_pub.publish(msg)
    
    def set_velocity(self, velocity: Vector3):
        """
        Update target velocity during execution.
        
        Args:
            velocity: New target velocity (vx, vy, vz) in m/s
        """
        self.target_velocity = self._clamp_velocity(velocity)
    
    def set_yaw(self, yaw: Optional[float] = None, yaw_rate: float = 0.0):
        """
        Update yaw control during execution.
        
        Args:
            yaw: Target yaw angle in radians (None = use yaw_rate)
            yaw_rate: Yaw rate in rad/s
        """
        self.target_yaw = yaw
        self.target_yaw_rate = yaw_rate
    
    def get_current_speed(self) -> float:
        """
        Get magnitude of current target velocity.
        
        Returns:
            float: Speed in m/s
        """
        return math.sqrt(
            self.target_velocity.x**2 + 
            self.target_velocity.y**2 + 
            self.target_velocity.z**2
        )
    
    def cancel(self) -> bool:
        """
        Cancel the velocity command (will hover).
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            # Send zero velocity to stop
            self.target_velocity = Vector3()
            self._send_raw_setpoint()
            self.logger.info("Velocity command cancelled, sending zero velocity")
            return super().cancel()
        return False
