#!/usr/bin/env python3
"""Arm/disarm primitive for drone control."""

import time
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

from .base_primitive import BasePrimitive, PrimitiveState


class ArmPrimitive(BasePrimitive):
    """
    Primitive for arming and disarming the drone.
    
    This primitive sends arm/disarm commands via MAVROS and monitors
    the arming state to confirm successful execution.
    """
    
    def __init__(self, node: Node, drone_namespace: str):
        """
        Initialize the arm primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
        """
        super().__init__(node, drone_namespace)
        
        # Create service client for arming
        arm_service_name = f"{drone_namespace}/mavros/cmd/arming"
        self.arm_client = node.create_client(CommandBool, arm_service_name)
        
        # Create service client for setting mode
        setmode_service_name = f"{drone_namespace}/mavros/set_mode"
        self.setmode_client = node.create_client(SetMode, setmode_service_name)
        
        # QoS profile for MAVROS topics (best effort to match MAVROS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to state topic
        state_topic = f"{drone_namespace}/mavros/state"
        self.state_sub = node.create_subscription(
            State,
            state_topic,
            self._state_callback,
            qos_profile
        )
        
        # State tracking
        self.current_state: Optional[State] = None
        self.target_armed_state = False
        self.command_sent_time = None
        self.timeout = 5.0  # seconds
        
        self.logger.info(f"ArmPrimitive initialized for {drone_namespace}")
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self.current_state = msg
        
    def execute(self, arm: bool = True, force: bool = False, timeout: float = 5.0) -> bool:
        """
        Execute arm or disarm command.
        
        Args:
            arm: True to arm, False to disarm
            force: Force operation even if safety checks fail
            timeout: Maximum time to wait for arming confirmation (seconds)
            
        Returns:
            bool: True if command was sent successfully, False otherwise
        """
        self.reset()
        self.target_armed_state = arm
        self.timeout = timeout
        
        # Wait for services to be available
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.set_error(f"Arming service not available: {self.arm_client.srv_name}")
            return False
        
        if not self.setmode_client.wait_for_service(timeout_sec=2.0):
            self.set_error(f"SetMode service not available: {self.setmode_client.srv_name}")
            return False
        
        # Check current state
        if self.current_state is None:
            self.set_error("No state information received from MAVROS")
            return False
        
        # If arming, set GUIDED mode first
        if arm and not self.current_state.guided:
            self.logger.info("Setting GUIDED mode before arming...")
            mode_request = SetMode.Request()
            mode_request.custom_mode = "GUIDED"
            
            try:
                mode_future = self.setmode_client.call_async(mode_request)
                rclpy.spin_until_future_complete(self.node, mode_future, timeout_sec=2.0)
                
                if mode_future.result() is not None:
                    mode_response = mode_future.result()
                    if not mode_response.mode_sent:
                        self.set_error("Failed to set GUIDED mode")
                        return False
                    self.logger.info("GUIDED mode set successfully")
                    # Wait a bit for mode to take effect
                    time.sleep(0.5)
                else:
                    self.set_error("SetMode service call timed out")
                    return False
            except Exception as e:
                self.set_error(f"Exception setting GUIDED mode: {str(e)}")
                return False
        
        # Check if already in target state
        if self.current_state.armed == arm:
            self.logger.info(f"Drone already in target armed state: {arm}")
            self.set_success()
            return True
        
        # Send arm/disarm command
        request = CommandBool.Request()
        request.value = arm
        
        self.logger.info(f"Sending {'arm' if arm else 'disarm'} command...")
        
        try:
            future = self.arm_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.state = PrimitiveState.EXECUTING
                    self.command_sent_time = time.time()
                    self.logger.info(f"{'Arm' if arm else 'Disarm'} command sent successfully")
                    return True
                else:
                    self.set_error(f"Arm command rejected: {response.result}")
                    return False
            else:
                self.set_error("Arm service call timed out")
                return False
                
        except Exception as e:
            self.set_error(f"Exception during arm command: {str(e)}")
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
                self.set_error(f"Arm/disarm timeout after {elapsed:.1f}s")
                return self.state
            
            # Update progress based on time
            self.progress = min(95.0, (elapsed / self.timeout) * 100.0)
        
        # Check if armed state matches target
        if self.current_state is not None:
            if self.current_state.armed == self.target_armed_state:
                self.set_success()
                return self.state
        
        return self.state
    
    def cancel(self) -> bool:
        """
        Cancel the arming operation.
        
        Note: Arming operations cannot really be cancelled once sent,
        but we can stop waiting for confirmation.
        
        Returns:
            bool: True if cancellation was successful
        """
        if self.state == PrimitiveState.EXECUTING:
            self.logger.warning("Cancelling arm primitive (stops waiting for confirmation)")
            return super().cancel()
        return False
