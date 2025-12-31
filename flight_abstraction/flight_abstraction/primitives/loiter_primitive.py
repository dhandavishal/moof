#!/usr/bin/env python3
"""Loiter primitive for holding position for a specified duration."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Point

from .base_primitive import BasePrimitive, PrimitiveState


class LoiterPrimitive(BasePrimitive):
    """
    Primitive for loitering at current or specified position.
    
    This primitive commands the drone to hold position for a specified
    duration. It can use either LOITER mode (GPS-based) or GUIDED mode
    with position setpoints.
    """
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the loiter primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create service client for setting mode
        setmode_service_name = f"{drone_namespace}/mavros/set_mode"
        self.setmode_client = node.create_client(SetMode, setmode_service_name)
        
        # Create publisher for position setpoint (for GUIDED mode loiter)
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
        self.loiter_position = Point()
        self.loiter_orientation = None
        self.loiter_radius = 0.0  # For LOITER mode
        self.duration = 0.0
        self.position_tolerance = 2.0  # meters
        self.use_loiter_mode = False  # False = GUIDED mode setpoints
        self.command_sent_time = None
        self.last_setpoint_time = 0.0
        self.setpoint_rate = 10.0  # Hz (lower rate for loiter)
        
        self.logger.info(f"LoiterPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_pose = msg
    
    def _calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calculate 3D distance between two points."""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def execute(self, duration: float, position: Optional[Point] = None,
                radius: float = 0.0, use_loiter_mode: bool = False,
                position_tolerance: float = 2.0) -> bool:
        """
        Execute loiter at current or specified position.
        
        Args:
            duration: How long to loiter in seconds (must be > 0)
            position: Target loiter position (None = current position)
            radius: Loiter radius in meters (for LOITER mode)
            use_loiter_mode: Use ArduPilot LOITER mode vs GUIDED setpoints
            position_tolerance: Maximum allowed drift (meters)
            
        Returns:
            bool: True if loiter started successfully, False otherwise
        """
        self.reset()
        self.duration = duration
        self.loiter_radius = radius
        self.use_loiter_mode = use_loiter_mode
        self.position_tolerance = position_tolerance
        
        # Validate duration
        if duration <= 0:
            self.set_error(f"Invalid loiter duration: {duration}s (must be > 0)")
            return False
        
        # Check if we have position information
        if self.current_pose is None:
            self.set_error("No position information available")
            return False
        
        # Check if armed
        if self.current_state is None or not self.current_state.armed:
            self.set_error("Drone must be armed to loiter")
            return False
        
        # Set loiter position
        if position is not None:
            self.loiter_position = position
        else:
            self.loiter_position.x = self.current_pose.pose.position.x
            self.loiter_position.y = self.current_pose.pose.position.y
            self.loiter_position.z = self.current_pose.pose.position.z
        
        self.loiter_orientation = self.current_pose.pose.orientation
        
        self.logger.info(f"Starting loiter at ({self.loiter_position.x:.2f}, "
                        f"{self.loiter_position.y:.2f}, {self.loiter_position.z:.2f}) "
                        f"for {duration:.1f}s")
        
        # Set mode if using LOITER mode
        if use_loiter_mode:
            if not self._set_mode("LOITER"):
                self.logger.warning("Failed to set LOITER mode, using GUIDED setpoints")
                self.use_loiter_mode = False
        
        self.state = PrimitiveState.EXECUTING
        self.command_sent_time = time.time()
        
        return True
    
    def _set_mode(self, mode: str) -> bool:
        """Set flight mode via MAVROS."""
        if not self.setmode_client.wait_for_service(timeout_sec=2.0):
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            future = self.setmode_client.call_async(request)
            
            wait_start = time.time()
            while not future.done() and (time.time() - wait_start) < 2.0:
                time.sleep(0.05)
            
            if future.done() and future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.logger.info(f"Mode set to {mode}")
                    return True
            return False
        except Exception:
            return False
    
    def update(self) -> PrimitiveState:
        """
        Update primitive state and check for completion.
        
        Returns:
            PrimitiveState: Current state of the primitive
        """
        if self.state != PrimitiveState.EXECUTING:
            return self.state
        
        current_time = time.time()
        
        # Check if duration completed
        if self.command_sent_time is not None:
            elapsed = current_time - self.command_sent_time
            
            if elapsed >= self.duration:
                self.logger.info(f"Loiter completed: {elapsed:.1f}s")
                self.set_success()
                return self.state
            
            # Update progress
            self.progress = min(95.0, (elapsed / self.duration) * 100.0)
        
        # Check position drift
        if self.current_pose is not None:
            drift = self._calculate_distance(
                self.current_pose.pose.position, 
                self.loiter_position
            )
            
            if drift > self.position_tolerance:
                self.logger.warning(f"Position drift: {drift:.2f}m > {self.position_tolerance:.2f}m")
        
        # Send position setpoint if using GUIDED mode
        if not self.use_loiter_mode:
            if (current_time - self.last_setpoint_time) >= (1.0 / self.setpoint_rate):
                self._send_setpoint()
                self.last_setpoint_time = current_time
        
        return self.state
    
    def _send_setpoint(self):
        """Send position setpoint to maintain loiter position."""
        msg = PoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Set loiter position
        msg.pose.position.x = self.loiter_position.x
        msg.pose.position.y = self.loiter_position.y
        msg.pose.position.z = self.loiter_position.z
        
        # Maintain orientation
        if self.loiter_orientation is not None:
            msg.pose.orientation = self.loiter_orientation
        else:
            msg.pose.orientation.w = 1.0
        
        self.setpoint_pub.publish(msg)
    
    def get_elapsed_time(self) -> float:
        """
        Get elapsed loiter time.
        
        Returns:
            float: Time elapsed in seconds
        """
        if self.command_sent_time is None:
            return 0.0
        return time.time() - self.command_sent_time
    
    def get_remaining_time(self) -> float:
        """
        Get remaining loiter time.
        
        Returns:
            float: Time remaining in seconds
        """
        return max(0.0, self.duration - self.get_elapsed_time())
    
    def get_position_drift(self) -> float:
        """
        Get current position drift from loiter point.
        
        Returns:
            float: Drift in meters
        """
        if self.current_pose is None:
            return 0.0
        return self._calculate_distance(
            self.current_pose.pose.position, 
            self.loiter_position
        )
    
    def extend_duration(self, additional_seconds: float):
        """
        Extend the loiter duration.
        
        Args:
            additional_seconds: Additional time to loiter
        """
        if self.state == PrimitiveState.EXECUTING:
            self.duration += additional_seconds
            self.logger.info(f"Loiter extended by {additional_seconds:.1f}s "
                           f"(new total: {self.duration:.1f}s)")
    
    def cancel(self) -> bool:
        """
        Cancel the loiter operation.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            elapsed = self.get_elapsed_time()
            self.logger.info(f"Loiter cancelled after {elapsed:.1f}s")
            
            # If in LOITER mode, switch back to GUIDED
            if self.use_loiter_mode:
                self._set_mode("GUIDED")
            
            return super().cancel()
        return False
