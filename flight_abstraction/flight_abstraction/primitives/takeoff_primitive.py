#!/usr/bin/env python3
"""Takeoff primitive for drone control."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    QoSDurabilityPolicy,
)
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

from .base_primitive import BasePrimitive, PrimitiveState


class TakeoffPrimitive(BasePrimitive):
    """
    Primitive for takeoff to a target altitude.
    
    This primitive commands the drone to takeoff and monitors altitude
    until the target is reached.
    """
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the takeoff primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create service clients
        takeoff_service_name = f"{drone_namespace}/mavros/cmd/takeoff"
        self.takeoff_client = node.create_client(CommandTOL, takeoff_service_name)
        
        setmode_service_name = f"{drone_namespace}/mavros/set_mode"
        self.setmode_client = node.create_client(SetMode, setmode_service_name)
        
        # QoS profiles for MAVROS topics
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        pose_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
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
        self.current_altitude = 0.0
        self.initial_altitude = 0.0
        self.target_altitude = 0.0
        self.acceptance_radius = 0.5  # meters
        self.command_sent_time = None
        self.timeout = 30.0  # seconds
        
        self.logger.info(f"TakeoffPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_altitude = msg.pose.position.z
        
    def execute(self, target_altitude: float, climb_rate: float = 1.0, timeout: float = 30.0) -> bool:
        """
        Execute takeoff to target altitude.
        
        Args:
            target_altitude: Target altitude in meters (relative to takeoff point)
            climb_rate: Desired climb rate in m/s (not used in current implementation)
            timeout: Maximum time to wait for takeoff completion (seconds)
            
        Returns:
            bool: True if takeoff command was sent successfully, False otherwise
        """
        self.reset()
        self.target_altitude = target_altitude
        self.timeout = timeout
        self.initial_altitude = self.current_altitude
        
        # Validate parameters
        if target_altitude <= 0.0:
            self.set_error(f"Invalid target altitude: {target_altitude}")
            return False
        
        if target_altitude > 100.0:  # Safety limit
            self.set_error(f"Target altitude too high: {target_altitude}m (max 100m)")
            return False
        
        # Check if we have state information
        if self.current_state is None:
            self.set_error("No state information received from MAVROS")
            return False
        
        # Check if drone is armed (don't wait, just check current state)
        if not self.current_state.armed:
            self.set_error("Drone must be armed before takeoff")
            return False
        
        # Set mode to GUIDED (required for ArduPilot)
        if not self._set_mode("GUIDED"):
            self.set_error("Failed to set GUIDED mode")
            return False
        
        # Wait for service to be available
        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.set_error(f"Takeoff service not available: {self.takeoff_client.srv_name}")
            return False
        
        # Send takeoff command
        request = CommandTOL.Request()
        request.altitude = target_altitude
        request.latitude = 0.0  # Not used for local frame
        request.longitude = 0.0
        request.min_pitch = 0.0
        request.yaw = 0.0
        
        self.logger.info(f"Sending takeoff command to altitude: {target_altitude}m")
        
        try:
            future = self.takeoff_client.call_async(request)
            
            # Wait for future without spinning executor
            wait_start = time.time()
            while not future.done() and (time.time() - wait_start) < 2.0:
                time.sleep(0.05)  # Yield thread
            
            if future.done() and future.result() is not None:
                response = future.result()
                if response.success:
                    self.state = PrimitiveState.EXECUTING
                    self.command_sent_time = time.time()
                    self.logger.info("Takeoff command sent successfully")
                    return True
                else:
                    self.set_error(f"Takeoff command rejected: {response.result}")
                    return False
            else:
                self.set_error("Takeoff service call timed out")
                return False
                
        except Exception as e:
            self.set_error(f"Exception during takeoff command: {str(e)}")
            return False
    
    def _set_mode(self, mode: str) -> bool:
        """
        Set flight mode via MAVROS.
        
        Args:
            mode: Flight mode string (e.g., "GUIDED", "AUTO")
            
        Returns:
            bool: True if mode was set successfully
        """
        if not self.setmode_client.wait_for_service(timeout_sec=2.0):
            self.logger.error(f"Set mode service not available")
            return False
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            future = self.setmode_client.call_async(request)

            # Wait for future without spinning executor
            wait_start = time.time()
            while not future.done() and (time.time() - wait_start) < 2.0:
                time.sleep(0.05)  # Yield thread
            
            if future.done() and future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.logger.info(f"Mode set to {mode}")
                    return True
                else:
                    self.logger.error(f"Failed to set mode to {mode}")
                    return False
            else:
                self.logger.error("Set mode service call timed out")
                return False
                
        except Exception as e:
            self.logger.error(f"Exception during set mode: {str(e)}")
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
                self.set_error(f"Takeoff timeout after {elapsed:.1f}s (altitude: {self.current_altitude:.2f}m)")
                return self.state
        
        # Calculate progress based on altitude
        altitude_gain = self.current_altitude - self.initial_altitude
        if self.target_altitude > 0:
            self.progress = min(95.0, (altitude_gain / self.target_altitude) * 100.0)
        
        # Check if target altitude reached
        altitude_error = abs(self.current_altitude - self.target_altitude)
        if altitude_error < self.acceptance_radius:
            self.logger.info(f"Target altitude reached: {self.current_altitude:.2f}m (target: {self.target_altitude:.2f}m)")
            self.set_success()
            return self.state
        
        # Log progress periodically
        if self.command_sent_time is not None:
            if int(time.time() - self.command_sent_time) % 2 == 0:  # Every 2 seconds
                self.logger.debug(f"Takeoff progress: altitude={self.current_altitude:.2f}m, target={self.target_altitude:.2f}m, progress={self.progress:.1f}%")
        
        return self.state
    
    def cancel(self) -> bool:
        """
        Cancel the takeoff operation.
        
        Note: This will transition to hover at current altitude.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            self.logger.warning(f"Cancelling takeoff at altitude: {self.current_altitude:.2f}m")
            # Note: In a real implementation, you might want to send a hover command
            return super().cancel()
        return False
