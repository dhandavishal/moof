#!/usr/bin/env python3
"""Arm/disarm primitive for drone control."""

import time
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

from .base_primitive import BasePrimitive, PrimitiveState


class ArmPrimitive(BasePrimitive):
    """
    Primitive for arming and disarming the drone.
    
    This primitive sends arm/disarm commands via MAVROS and monitors
    the arming state to confirm successful execution.
    """
    
    def __init__(self, node: Node, drone_namespace: str, callback_group=None):
        """
        Initialize the arm primitive.
        
        Args:
            node: ROS2 node for communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
            callback_group: Optional callback group for subscriptions
        """
        super().__init__(node, drone_namespace)
        
        # Create service client for arming
        arm_service_name = f"{drone_namespace}/mavros/cmd/arming"
        self.arm_client = node.create_client(CommandBool, arm_service_name)
        
        # Create service client for setting mode
        setmode_service_name = f"{drone_namespace}/mavros/set_mode"
        self.setmode_client = node.create_client(SetMode, setmode_service_name)
        
        # QoS profile for MAVROS state topic (RELIABLE + TRANSIENT_LOCAL)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to state topic
        state_topic = f"{drone_namespace}/mavros/state"
        self.state_sub = node.create_subscription(
            State,
            state_topic,
            self._state_callback,
            qos_profile,
            callback_group=callback_group
        )
        
        # State tracking
        self.current_state: Optional[State] = None
        self.target_armed_state = False
        self.command_sent_time = None
        self.timeout = 5.0  # seconds
        
        self.logger.info(f"ArmPrimitive initialized for {drone_namespace}")
        self._callback_count = 0
        
    def _state_callback(self, msg: State):
        """Callback for MAVROS state updates."""
        self._callback_count += 1
        if self._callback_count <= 3:
            self.logger.info(f"ArmPrimitive state callback #{self._callback_count}: armed={msg.armed}, connected={msg.connected}")
        
        old_armed = self.current_state.armed if self.current_state else None
        self.current_state = msg
        
        # Log when armed state changes
        if old_armed is not None and old_armed != msg.armed:
            self.logger.info(f"Armed state changed: {old_armed} -> {msg.armed}")
        
        # Log every callback during EXECUTING state to debug subscription issues
        if self.state == PrimitiveState.EXECUTING:
            self.logger.info(f"State callback during EXECUTING: armed={msg.armed}, mode={msg.mode}, connected={msg.connected}")
        
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
            self.set_error("No state information received from MAVROS (is MAVROS running?)")
            return False
        
        # If arming, set GUIDED mode first
        if arm and not self.current_state.guided:
            self.logger.info("Setting GUIDED mode before arming...")
            mode_request = SetMode.Request()
            mode_request.custom_mode = "GUIDED"
            
            try:
                mode_future = self.setmode_client.call_async(mode_request)
                
                # Wait for future without spinning executor
                wait_start = time.time()
                while not mode_future.done() and (time.time() - wait_start) < 2.0:
                    time.sleep(0.05)  # Yield thread
                
                if mode_future.done() and mode_future.result() is not None:
                    mode_response = mode_future.result()
                    if not mode_response.mode_sent:
                        self.set_error("Failed to set GUIDED mode")
                        return False
                    self.logger.info("GUIDED mode set successfully")
                    
                    # Wait for mode to take effect (this sleep is fine)
                    time.sleep(1.0)
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
            
            # Wait for future without spinning executor
            wait_start = time.time()
            while not future.done() and (time.time() - wait_start) < 2.0:
                time.sleep(0.05)  # Yield thread
            
            if future.done() and future.result() is not None:
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
                # Service timed out - but command may have been received!
                # This happens due to MAVROS future_error bug
                self.logger.warn("Arm service call timed out, but command may have been sent")
                self.logger.warn("Will check actual armed state to verify...")
                
                # Give ArduPilot more time to process the command (check multiple times)
                for i in range(6):  # Check for 3 seconds total
                    time.sleep(0.5)
                    if self.current_state and self.current_state.armed == arm:
                        self.logger.info(f"Command succeeded despite timeout! Armed state is now: {arm} (after {(i+1)*0.5}s)")
                        self.state = PrimitiveState.SUCCESS
                        return True
                
                # Still not armed after 3 seconds, start executing and let update() monitor
                self.logger.info("State not changed after 3s, will monitor in update()")
                self.state = PrimitiveState.EXECUTING
                self.command_sent_time = time.time()
                return True
                
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
                self.set_error(f"Arm/disarm timeout after {self.timeout:.1f}s")
                return self.state
            
            # Update progress based on time
            self.progress = min(95.0, (elapsed / self.timeout) * 100.0)
        
        # Check if armed state matches target
        if self.current_state is not None:
            current_armed = self.current_state.armed
            if current_armed == self.target_armed_state:
                self.logger.info(f"Arm state confirmed: armed={current_armed}")
                self.set_success()
                return self.state
            else:
                # Log current state for debugging
                if int(time.time() * 2) % 2 == 0:  # Log every 0.5s
                    self.logger.debug(f"Waiting for arm state change: current={current_armed}, target={self.target_armed_state}")
        
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
