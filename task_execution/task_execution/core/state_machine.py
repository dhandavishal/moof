#!/usr/bin/env python3
"""
Mission state machine for ROS2.

Implements mission lifecycle state management with validated transitions
and history tracking.
"""

from enum import Enum
from typing import Optional, Callable, Dict, List
from collections import deque
from rclpy.node import Node


class MissionState(Enum):
    """Mission lifecycle states"""
    IDLE = "idle"
    VALIDATING = "validating"
    EXECUTING = "executing"
    PAUSED = "paused"
    COMPLETED = "completed"
    ABORTED = "aborted"
    EMERGENCY = "emergency"


class StateTransition:
    """Record of a state transition"""
    def __init__(self, from_state, to_state, reason, timestamp):
        self.from_state = from_state
        self.to_state = to_state
        self.reason = reason
        self.timestamp = timestamp


class StateMachine:
    """
    Mission state machine with validated transitions and history tracking.
    """
    
    def __init__(self, node: Node, max_history: int = 100):
        """
        Initialize state machine.
        
        Args:
            node: ROS2 node instance
            max_history: Maximum number of transitions to keep in history
        """
        self.node = node
        self.current_state = MissionState.IDLE
        self.previous_state = None
        self.transition_history = deque(maxlen=max_history)
        
        # Callbacks: state -> list of callback functions
        self.entry_callbacks: Dict[MissionState, List[Callable]] = {}
        self.exit_callbacks: Dict[MissionState, List[Callable]] = {}
        
        # Define valid state transitions
        self._define_transitions()
        
        self.node.get_logger().info("State machine initialized in IDLE state")
    
    def _define_transitions(self):
        """Define valid state transition graph"""
        self.valid_transitions = {
            MissionState.IDLE: [
                MissionState.VALIDATING,
                MissionState.EMERGENCY
            ],
            MissionState.VALIDATING: [
                MissionState.EXECUTING,
                MissionState.ABORTED,
                MissionState.EMERGENCY
            ],
            MissionState.EXECUTING: [
                MissionState.PAUSED,
                MissionState.COMPLETED,
                MissionState.ABORTED,
                MissionState.EMERGENCY
            ],
            MissionState.PAUSED: [
                MissionState.EXECUTING,
                MissionState.ABORTED,
                MissionState.EMERGENCY
            ],
            MissionState.COMPLETED: [
                MissionState.IDLE
            ],
            MissionState.ABORTED: [
                MissionState.IDLE
            ],
            MissionState.EMERGENCY: [
                MissionState.IDLE,
                MissionState.ABORTED
            ]
        }
    
    def transition_to(self, new_state: MissionState, reason: str = "") -> bool:
        """
        Attempt state transition with validation.
        
        Args:
            new_state: Target state
            reason: Human-readable reason for transition
            
        Returns:
            True if transition successful, False otherwise
        """
        # Validate transition
        if not self._is_valid_transition(new_state):
            self.node.get_logger().error(
                f"Invalid state transition: {self.current_state.value} -> {new_state.value}"
            )
            return False
        
        # Execute exit callbacks for current state
        self._execute_exit_callbacks(self.current_state)
        
        # Perform transition
        self.previous_state = self.current_state
        self.current_state = new_state
        
        # Record transition
        transition = StateTransition(
            from_state=self.previous_state,
            to_state=new_state,
            reason=reason,
            timestamp=self.node.get_clock().now().nanoseconds / 1e9
        )
        self.transition_history.append(transition)
        
        self.node.get_logger().info(
            f"State transition: {self.previous_state.value} -> {new_state.value} "
            f"(reason: {reason})"
        )
        
        # Execute entry callbacks for new state
        self._execute_entry_callbacks(new_state)
        
        return True
    
    def _is_valid_transition(self, target_state: MissionState) -> bool:
        """Check if transition is allowed"""
        if self.current_state not in self.valid_transitions:
            return False
        
        # Emergency state can be reached from anywhere
        if target_state == MissionState.EMERGENCY:
            return True
        
        return target_state in self.valid_transitions[self.current_state]
    
    def register_entry_callback(self, state: MissionState, callback: Callable):
        """Register function to call when entering a state"""
        if state not in self.entry_callbacks:
            self.entry_callbacks[state] = []
        self.entry_callbacks[state].append(callback)
    
    def register_exit_callback(self, state: MissionState, callback: Callable):
        """Register function to call when exiting a state"""
        if state not in self.exit_callbacks:
            self.exit_callbacks[state] = []
        self.exit_callbacks[state].append(callback)
    
    def _execute_entry_callbacks(self, state: MissionState):
        """Execute all entry callbacks for state"""
        if state in self.entry_callbacks:
            for callback in self.entry_callbacks[state]:
                try:
                    callback()
                except Exception as e:
                    self.node.get_logger().error(f"Entry callback failed for {state.value}: {e}")
    
    def _execute_exit_callbacks(self, state: MissionState):
        """Execute all exit callbacks for state"""
        if state in self.exit_callbacks:
            for callback in self.exit_callbacks[state]:
                try:
                    callback()
                except Exception as e:
                    self.node.get_logger().error(f"Exit callback failed for {state.value}: {e}")
    
    def get_state(self) -> MissionState:
        """Get current state"""
        return self.current_state
    
    def get_history(self, n: int = 10) -> List[StateTransition]:
        """Get last n state transitions"""
        return list(self.transition_history)[-n:]
    
    def is_state(self, state: MissionState) -> bool:
        """Check if current state matches given state"""
        return self.current_state == state
