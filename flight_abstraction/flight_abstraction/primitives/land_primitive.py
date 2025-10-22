#!/usr/bin/env python3
"""Land primitive for drone control."""

import time
import math
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandTOL, SetMode
from mavros_msgs.msg import State, ExtendedState
from geometry_msgs.msg import PoseStamped, Point

from .base_primitive import BasePrimitive, PrimitiveState


class LandPrimitive(BasePrimitive):
    """
    Primitive for landing the drone.
    
    This primitive commands the drone to land and monitors the landing
    state until touchdown is confirmed.
    """
    
    def __init__(self, node: Node, drone_namespace: str):
        """
        Initialize the land primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
        """
        super().__init__(node, drone_namespace)
        
        # Create service clients
        land_service_name = f"{drone_namespace}/mavros/cmd/land"
        self.land_client = node.create_client(CommandTOL, land_service_name)
        
        setmode_service_name = f"{drone_namespace}/mavros/set_mode"
        self.setmode_client = node.create_client(SetMode, setmode_service_name)
        
        # QoS profile for MAVROS topics (reliable matches MAVROS defaults)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to state, extended state, and local position
        state_topic = f"{drone_namespace}/mavros/state"
        self.state_sub = node.create_subscription(
            State,
            state_topic,
            self._state_callback,
            qos_profile
        )
        
        extended_state_topic = f"{drone_namespace}/mavros/extended_state"
        self.extended_state_sub = node.create_subscription(
            ExtendedState,
            extended_state_topic,
            self._extended_state_callback,
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
        self.current_extended_state: Optional[ExtendedState] = None
        self.current_altitude = 0.0
        self.initial_altitude = 0.0
        self.target_position: Optional[Point] = None
        self.descent_rate = 0.5  # m/s
        self.ground_altitude_threshold = 0.3  # meters
        self.command_sent_time = None
        self.timeout = 60.0  # seconds
        
        self.logger.info(f"LandPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def _extended_state_callback(self, msg: ExtendedState):
        """Callback for MAVROS extended state updates."""
        self.current_extended_state = msg
        
    def _local_pos_callback(self, msg: PoseStamped):
        """Callback for local position updates."""
        self.current_altitude = msg.pose.position.z
        
    def execute(self, target_position: Optional[Point] = None, 
                descent_rate: float = 0.5, timeout: float = 60.0) -> bool:
        """
        Execute landing operation.
        
        Args:
            target_position: Target landing position (if None, land at current position)
            descent_rate: Desired descent rate in m/s (not used in current implementation)
            timeout: Maximum time to wait for landing completion (seconds)
            
        Returns:
            bool: True if land command was sent successfully, False otherwise
        """
        self.reset()
        self.target_position = target_position
        self.descent_rate = descent_rate
        self.timeout = timeout
        self.initial_altitude = self.current_altitude
        
        # Validate parameters
        if descent_rate <= 0.0 or descent_rate > 2.0:
            self.set_error(f"Invalid descent rate: {descent_rate} (must be 0-2 m/s)")
            return False
        
        # Check if armed
        if self.current_state is None or not self.current_state.armed:
            self.logger.warning("Drone not armed, but attempting to land anyway")
        
        # Check current altitude
        if self.current_altitude < self.ground_altitude_threshold:
            self.logger.info(f"Already on ground (altitude: {self.current_altitude:.2f}m)")
            self.set_success()
            return True
        
        # Set mode to LAND (ArduPilot)
        if not self._set_mode("LAND"):
            self.logger.warning("Failed to set LAND mode, trying land command anyway")
        
        # Wait for service to be available
        if not self.land_client.wait_for_service(timeout_sec=2.0):
            self.set_error(f"Land service not available: {self.land_client.srv_name}")
            return False
        
        # Send land command
        request = CommandTOL.Request()
        if target_position is not None:
            request.latitude = target_position.x  # Note: These might need conversion
            request.longitude = target_position.y
            request.altitude = target_position.z
        else:
            request.latitude = 0.0
            request.longitude = 0.0
            request.altitude = 0.0
        
        self.logger.info(f"Sending land command from altitude: {self.current_altitude:.2f}m")
        
        try:
            future = self.land_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.state = PrimitiveState.EXECUTING
                    self.command_sent_time = time.time()
                    self.logger.info("Land command sent successfully")
                    return True
                else:
                    self.set_error(f"Land command rejected: {response.result}")
                    return False
            else:
                self.set_error("Land service call timed out")
                return False
                
        except Exception as e:
            self.set_error(f"Exception during land command: {str(e)}")
            return False
    
    def _set_mode(self, mode: str) -> bool:
        """
        Set flight mode via MAVROS.
        
        Args:
            mode: Flight mode string (e.g., "LAND", "RTL")
            
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
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.result() is not None:
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
                self.set_error(f"Landing timeout after {elapsed:.1f}s (altitude: {self.current_altitude:.2f}m)")
                return self.state
        
        # Calculate progress based on altitude decrease
        altitude_descent = self.initial_altitude - self.current_altitude
        if self.initial_altitude > 0:
            self.progress = min(95.0, (altitude_descent / self.initial_altitude) * 100.0)
        
        # Check if landed using extended state
        if self.current_extended_state is not None:
            # ExtendedState.LANDED_STATE_ON_GROUND = 1
            if self.current_extended_state.landed_state == 1:
                self.logger.info(f"Landing confirmed by extended state")
                self.set_success()
                return self.state
        
        # Fallback: Check altitude
        if self.current_altitude < self.ground_altitude_threshold:
            self.logger.info(f"Landing confirmed by altitude: {self.current_altitude:.2f}m")
            self.set_success()
            return self.state
        
        # Check if disarmed (indicates successful landing)
        if self.current_state is not None and not self.current_state.armed:
            self.logger.info("Drone disarmed - landing complete")
            self.set_success()
            return self.state
        
        # Log progress periodically
        if self.command_sent_time is not None:
            elapsed = time.time() - self.command_sent_time
            if int(elapsed) % 2 == 0:  # Every 2 seconds
                self.logger.debug(f"Landing progress: altitude={self.current_altitude:.2f}m, progress={self.progress:.1f}%")
        
        return self.state
    
    def cancel(self) -> bool:
        """
        Cancel the landing operation.
        
        Note: Landing is difficult to cancel safely. This will attempt to
        switch back to GUIDED mode and hover.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            self.logger.warning(f"Attempting to cancel landing at altitude: {self.current_altitude:.2f}m")
            
            # Try to switch back to GUIDED mode
            if self._set_mode("GUIDED"):
                self.logger.info("Switched to GUIDED mode - landing cancelled")
                return super().cancel()
            else:
                self.logger.error("Failed to switch to GUIDED mode - landing will continue")
                return False
        return False
