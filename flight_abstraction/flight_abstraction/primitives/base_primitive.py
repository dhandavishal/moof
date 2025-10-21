#!/usr/bin/env python3
"""Base primitive class for all flight operations."""

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any, Dict, Optional
import rclpy
from rclpy.node import Node


class PrimitiveState(Enum):
    """States for primitive execution."""
    IDLE = 0
    EXECUTING = 1
    SUCCESS = 2
    FAILED = 3
    CANCELLED = 4


class BasePrimitive(ABC):
    """
    Abstract base class for all flight primitives.
    
    Each primitive represents a discrete flight operation (arm, takeoff, goto, land, etc.)
    and provides a standardized interface for execution, cancellation, and status monitoring.
    """
    
    def __init__(self, node: Node, drone_namespace: str):
        """
        Initialize the base primitive.
        
        Args:
            node: ROS2 node for logging and communication
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
        """
        self.node = node
        self.drone_namespace = drone_namespace
        self.state = PrimitiveState.IDLE
        self.error_message = ""
        self.progress = 0.0
        
        # Create logger with primitive name
        self.logger = node.get_logger().get_child(self.__class__.__name__)
        
    @abstractmethod
    def execute(self, **kwargs) -> bool:
        """
        Execute the primitive with given parameters.
        
        This method should be implemented by each concrete primitive to perform
        the specific flight operation. It should be non-blocking or contain
        appropriate timeout mechanisms.
        
        Args:
            **kwargs: Primitive-specific parameters
            
        Returns:
            bool: True if execution started successfully, False otherwise
        """
        pass
    
    @abstractmethod
    def update(self) -> PrimitiveState:
        """
        Update the primitive state and check for completion.
        
        This method should be called periodically to check if the primitive
        has completed execution. It should update self.state and self.progress.
        
        Returns:
            PrimitiveState: Current state of the primitive
        """
        pass
    
    def cancel(self) -> bool:
        """
        Cancel the primitive execution.
        
        This method can be overridden by primitives that support cancellation.
        Default implementation just sets state to CANCELLED.
        
        Returns:
            bool: True if cancellation was successful, False otherwise
        """
        if self.state == PrimitiveState.EXECUTING:
            self.state = PrimitiveState.CANCELLED
            self.logger.info(f"Primitive cancelled: {self.__class__.__name__}")
            return True
        return False
    
    def get_state(self) -> PrimitiveState:
        """
        Get current state of the primitive.
        
        Returns:
            PrimitiveState: Current state
        """
        return self.state
    
    def get_progress(self) -> float:
        """
        Get execution progress as a percentage.
        
        Returns:
            float: Progress from 0.0 to 100.0
        """
        return self.progress
    
    def get_error_message(self) -> str:
        """
        Get error message if primitive failed.
        
        Returns:
            str: Error message or empty string if no error
        """
        return self.error_message
    
    def reset(self):
        """Reset the primitive to initial state."""
        self.state = PrimitiveState.IDLE
        self.error_message = ""
        self.progress = 0.0
        
    def get_status(self) -> Dict[str, Any]:
        """
        Get comprehensive status dictionary.
        
        Returns:
            dict: Status information including state, progress, and errors
        """
        return {
            'primitive': self.__class__.__name__,
            'state': self.state.name,
            'progress': self.progress,
            'error_message': self.error_message,
            'drone_namespace': self.drone_namespace,
        }
    
    def set_error(self, message: str):
        """
        Set error state with message.
        
        Args:
            message: Error description
        """
        self.state = PrimitiveState.FAILED
        self.error_message = message
        self.logger.error(f"Primitive failed: {message}")
    
    def set_success(self):
        """Set primitive state to success."""
        self.state = PrimitiveState.SUCCESS
        self.progress = 100.0
        self.logger.info(f"Primitive completed successfully: {self.__class__.__name__}")
