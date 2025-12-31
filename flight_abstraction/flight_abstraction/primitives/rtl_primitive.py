#!/usr/bin/env python3
"""Return to Launch (RTL) primitive for drone control."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State, HomePosition, ExtendedState
from geometry_msgs.msg import PoseStamped, Point

from .base_primitive import BasePrimitive, PrimitiveState


class RTLPrimitive(BasePrimitive):
    """
    Primitive for Return to Launch (RTL) operation.
    
    This primitive commands the drone to return to its home/launch position
    by setting RTL mode in ArduPilot. The drone will climb to RTL altitude,
    fly to home position, loiter briefly, then land.
    """
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the RTL primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create service client for setting mode
        setmode_service_name = f"{drone_namespace}/mavros/set_mode"
        self.setmode_client = node.create_client(SetMode, setmode_service_name)
        
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
        
        # Subscribe to state, extended state, position, and home
        state_topic = f"{drone_namespace}/mavros/state"
        self.state_sub = node.create_subscription(
            State,
            state_topic,
            self._state_callback,
            state_qos,
            callback_group=callback_group
        )
        
        extended_state_topic = f"{drone_namespace}/mavros/extended_state"
        self.extended_state_sub = node.create_subscription(
            ExtendedState,
            extended_state_topic,
            self._extended_state_callback,
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
        
        home_topic = f"{drone_namespace}/mavros/home_position/home"
        self.home_sub = node.create_subscription(
            HomePosition,
            home_topic,
            self._home_callback,
            state_qos,
            callback_group=callback_group
        )
        
        # State tracking
        self.current_state: Optional[State] = None
        self.current_extended_state: Optional[ExtendedState] = None
        self.current_position = Point()
        self.home_position: Optional[Point] = None
        self.initial_distance = 0.0
        self.command_sent_time = None
        self.timeout = 300.0  # 5 minutes for RTL
        self.land_detection_threshold = 0.3  # meters
        
        self.logger.info(f"RTLPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _extended_state_callback(self, msg: ExtendedState):
        """Callback for extended state updates."""
        self.current_extended_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_position.x = msg.pose.position.x
        self.current_position.y = msg.pose.position.y
        self.current_position.z = msg.pose.position.z
        
    def _home_callback(self, msg: HomePosition):
        """Callback for home position updates."""
        # Home position is typically at origin in local frame
        self.home_position = Point()
        self.home_position.x = msg.position.x
        self.home_position.y = msg.position.y
        self.home_position.z = 0.0  # Ground level
        
    def _calculate_distance_2d(self, p1: Point, p2: Point) -> float:
        """Calculate 2D horizontal distance between two points."""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        return math.sqrt(dx*dx + dy*dy)
    
    def _calculate_distance_3d(self, p1: Point, p2: Point) -> float:
        """Calculate 3D distance between two points."""
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def execute(self, timeout: float = 300.0) -> bool:
        """
        Execute Return to Launch.
        
        Args:
            timeout: Maximum time to complete RTL (seconds)
            
        Returns:
            bool: True if RTL started successfully, False otherwise
        """
        self.reset()
        self.timeout = timeout
        
        # Wait for service to be available
        if not self.setmode_client.wait_for_service(timeout_sec=2.0):
            self.set_error(f"SetMode service not available")
            return False
        
        # Check current state
        if self.current_state is None:
            self.set_error("No state information received from MAVROS")
            return False
        
        # Check if armed
        if not self.current_state.armed:
            self.set_error("Drone must be armed for RTL")
            return False
        
        # Calculate initial distance to home
        if self.home_position is not None:
            self.initial_distance = self._calculate_distance_2d(
                self.current_position, self.home_position
            )
        else:
            # Assume home is at origin
            self.home_position = Point()
            self.initial_distance = self._calculate_distance_2d(
                self.current_position, self.home_position
            )
        
        self.logger.info(f"Starting RTL from distance: {self.initial_distance:.1f}m")
        
        # Set RTL mode
        request = SetMode.Request()
        request.custom_mode = "RTL"
        
        try:
            future = self.setmode_client.call_async(request)
            
            # Wait for response
            wait_start = time.time()
            while not future.done() and (time.time() - wait_start) < 2.0:
                time.sleep(0.05)
            
            if future.done() and future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.state = PrimitiveState.EXECUTING
                    self.command_sent_time = time.time()
                    self.logger.info("RTL mode set successfully")
                    return True
                else:
                    self.set_error("Failed to set RTL mode")
                    return False
            else:
                self.set_error("SetMode service call timed out")
                return False
                
        except Exception as e:
            self.set_error(f"Exception during RTL: {str(e)}")
            return False
    
    def update(self) -> PrimitiveState:
        """
        Update primitive state and check for completion.
        
        Returns:
            PrimitiveState: Current state of the primitive
        """
        if self.state != PrimitiveState.EXECUTING:
            return self.state
        
        # Check timeout
        if self.command_sent_time is not None:
            elapsed = time.time() - self.command_sent_time
            if elapsed > self.timeout:
                self.set_error(f"RTL timeout after {elapsed:.1f}s")
                return self.state
        
        # Check if mode changed away from RTL
        if self.current_state is not None and self.current_state.mode != "RTL":
            # Mode changed, might have landed and disarmed
            if not self.current_state.armed:
                self.logger.info("RTL completed: drone disarmed")
                self.set_success()
                return self.state
            else:
                self.logger.warning(f"RTL mode changed to {self.current_state.mode}")
        
        # Calculate distance to home
        if self.home_position is not None:
            distance = self._calculate_distance_2d(self.current_position, self.home_position)
            
            # Calculate progress
            if self.initial_distance > 0.1:
                progress_ratio = 1.0 - (distance / self.initial_distance)
                self.progress = min(95.0, max(0.0, progress_ratio * 100.0))
        
        # Check if landed
        if self.current_extended_state is not None:
            if self.current_extended_state.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
                self.logger.info("RTL completed: landed")
                self.set_success()
                return self.state
        
        # Fallback: check altitude
        if self.current_position.z < self.land_detection_threshold:
            self.logger.info(f"RTL completed: altitude {self.current_position.z:.2f}m")
            self.set_success()
            return self.state
        
        return self.state
    
    def get_distance_to_home(self) -> float:
        """
        Get current distance to home position.
        
        Returns:
            float: Distance in meters (2D horizontal)
        """
        if self.home_position is None:
            return 0.0
        return self._calculate_distance_2d(self.current_position, self.home_position)
    
    def cancel(self) -> bool:
        """
        Cancel RTL operation.
        
        Note: This will set GUIDED mode to stop RTL and hover.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            self.logger.warning("Cancelling RTL, setting GUIDED mode")
            
            # Set GUIDED mode to stop RTL
            request = SetMode.Request()
            request.custom_mode = "GUIDED"
            
            try:
                future = self.setmode_client.call_async(request)
                
                wait_start = time.time()
                while not future.done() and (time.time() - wait_start) < 2.0:
                    time.sleep(0.05)
                
                if future.done() and future.result() is not None:
                    if future.result().mode_sent:
                        self.logger.info("GUIDED mode set, RTL cancelled")
                    else:
                        self.logger.warning("Failed to set GUIDED mode during cancel")
            except Exception as e:
                self.logger.error(f"Exception during RTL cancel: {str(e)}")
            
            return super().cancel()
        return False
