#!/usr/bin/env python3
"""
Base executor class for task execution.

This module provides the abstract base class for all task executors.
Executors generate PrimitiveCommand messages for the FAL to execute.
"""

import rclpy
from abc import ABC, abstractmethod
from typing import List, Dict, Any
import uuid
import math

from multi_drone_msgs.msg import PrimitiveCommand, PrimitiveStatus
from geometry_msgs.msg import Point, Quaternion


class BaseExecutor(ABC):
    """
    Abstract base class for task executors.
    
    Executors translate high-level task definitions into sequences of
    PrimitiveCommand messages that the Flight Abstraction Layer (FAL)
    can execute.
    
    Key Architectural Pattern:
    - TEE generates PrimitiveCommand messages (not FAL primitive instances)
    - Messages are published to FAL via ROS topics
    - FAL receives messages and instantiates actual primitive objects
    - This maintains proper separation of concerns
    """
    
    def __init__(self, config: Dict):
        """
        Initialize executor with configuration.
        
        Args:
            config: Configuration dictionary from YAML containing:
                   - drone: Drone-specific parameters (speeds, limits, etc.)
                   - tasks: Task-specific parameters (tolerances, etc.)
        """
        self.config = config
        self.drone_config = config.get('drone', {})
        self.task_config = config.get('tasks', {})
        
        # Create a simple logger (executors are not nodes, so we use print-based logging)
        self._logger_name = self.__class__.__name__
        
        # Default parameters
        self.default_cruise_speed = self.drone_config.get('cruise_speed', 2.0)  # m/s
        self.default_climb_rate = self.drone_config.get('max_climb_rate', 1.0)  # m/s
        self.default_acceptance_radius = self.task_config.get('waypoint', {}).get('acceptance_radius', 1.0)  # m
        
        self._log_info(f"{self._logger_name} initialized")
    
    def _log_info(self, message: str):
        """Log info message."""
        print(f"[INFO] [{self._logger_name}] {message}")
    
    def _log_warn(self, message: str):
        """Log warning message."""
        print(f"[WARN] [{self._logger_name}] {message}")
    
    def _log_error(self, message: str):
        """Log error message."""
        print(f"[ERROR] [{self._logger_name}] {message}")
    
    def _log_debug(self, message: str):
        """Log debug message."""
        print(f"[DEBUG] [{self._logger_name}] {message}")
        
    @abstractmethod
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Convert task parameters into primitive command sequence.
        
        This method must be implemented by all concrete executors.
        It defines the logic for breaking down a task into primitive commands.
        
        Args:
            parameters: Task-specific parameters
            
        Returns:
            List of PrimitiveCommand messages for FAL to execute
        """
        pass
    
    # ==================================================================
    # Primitive Command Factory Methods
    # ==================================================================
    
    def create_goto_primitive(self, 
                             position: Point,
                             velocity: float = None,
                             acceptance_radius: float = None,
                             yaw: float = None,
                             timeout: float = 300.0) -> PrimitiveCommand:
        """
        Create a goto (waypoint navigation) primitive command.
        
        Args:
            position: Target position (Point in map frame)
            velocity: Travel velocity in m/s (None = use default)
            acceptance_radius: Acceptance radius in meters (None = use default)
            yaw: Desired yaw angle in radians (optional)
            timeout: Maximum execution time in seconds
            
        Returns:
            PrimitiveCommand message ready to send to FAL
        """
        if velocity is None:
            velocity = self.default_cruise_speed
        
        if acceptance_radius is None:
            acceptance_radius = self.default_acceptance_radius
        
        cmd = PrimitiveCommand()
        cmd.command_id = str(uuid.uuid4())
        cmd.primitive_type = 'goto'
        cmd.target_position = position
        cmd.velocity = velocity
        cmd.acceptance_radius = acceptance_radius
        cmd.blocking = True
        cmd.frame_id = 'map'
        cmd.timeout = timeout
        
        if yaw is not None:
            # Convert yaw to quaternion
            cmd.target_orientation = self._yaw_to_quaternion(yaw)
        else:
            cmd.target_orientation = Quaternion()  # Identity quaternion
        
        return cmd
    
    def create_loiter_primitive(self, 
                               position: Point,
                               duration: float = 10.0,
                               radius: float = 5.0,
                               timeout: float = None) -> PrimitiveCommand:
        """
        Create a loiter (hold position/orbit) primitive command.
        
        Args:
            position: Loiter center position
            duration: Loiter duration in seconds
            radius: Loiter radius in meters (0 = hold position)
            timeout: Maximum execution time (None = duration + 10s buffer)
            
        Returns:
            PrimitiveCommand message
        """
        if timeout is None:
            timeout = duration + 10.0
        
        cmd = PrimitiveCommand()
        cmd.command_id = str(uuid.uuid4())
        cmd.primitive_type = 'loiter'
        cmd.target_position = position
        cmd.loiter_radius = radius
        cmd.loiter_duration = duration
        cmd.blocking = True
        cmd.frame_id = 'map'
        cmd.timeout = timeout
        cmd.target_orientation = Quaternion()  # Identity
        
        return cmd
    
    def create_takeoff_primitive(self, 
                                target_altitude: float,
                                climb_rate: float = None,
                                timeout: float = None) -> PrimitiveCommand:
        """
        Create takeoff primitive command.
        
        Args:
            target_altitude: Target altitude in meters AGL
            climb_rate: Climb rate in m/s (None = use default)
            timeout: Maximum execution time (None = auto-calculate)
            
        Returns:
            PrimitiveCommand message
        """
        if climb_rate is None:
            climb_rate = self.default_climb_rate
        
        if timeout is None:
            # Calculate timeout: altitude / climb_rate + 20s buffer
            timeout = max(target_altitude / climb_rate + 20.0, 30.0)
        
        cmd = PrimitiveCommand()
        cmd.command_id = str(uuid.uuid4())
        cmd.primitive_type = 'takeoff'
        
        # Target position is just altitude (z-axis)
        cmd.target_position = Point()
        cmd.target_position.x = 0.0
        cmd.target_position.y = 0.0
        cmd.target_position.z = target_altitude
        
        cmd.climb_rate = climb_rate
        cmd.blocking = True
        cmd.frame_id = 'map'
        cmd.timeout = timeout
        cmd.target_orientation = Quaternion()  # Identity
        
        return cmd
    
    def create_land_primitive(self, 
                             position: Point = None,
                             timeout: float = 60.0) -> PrimitiveCommand:
        """
        Create landing primitive command.
        
        Args:
            position: Optional landing position (None = land at current position)
            timeout: Maximum execution time in seconds
            
        Returns:
            PrimitiveCommand message
        """
        cmd = PrimitiveCommand()
        cmd.command_id = str(uuid.uuid4())
        cmd.primitive_type = 'land'
        
        if position is not None:
            cmd.target_position = position
        else:
            # Empty position = land at current location
            cmd.target_position = Point()
        
        cmd.blocking = True
        cmd.frame_id = 'map'
        cmd.timeout = timeout
        cmd.target_orientation = Quaternion()  # Identity
        
        return cmd
    
    def create_rtl_primitive(self, timeout: float = 300.0) -> PrimitiveCommand:
        """
        Create return-to-launch primitive command.
        
        Args:
            timeout: Maximum execution time in seconds (default 5 minutes)
            
        Returns:
            PrimitiveCommand message
        """
        cmd = PrimitiveCommand()
        cmd.command_id = str(uuid.uuid4())
        cmd.primitive_type = 'rtl'
        cmd.blocking = True
        cmd.timeout = timeout
        cmd.target_position = Point()  # RTL doesn't need position
        cmd.target_orientation = Quaternion()  # Identity
        cmd.frame_id = 'map'
        
        return cmd
    
    # ==================================================================
    # Utility Methods
    # ==================================================================
    
    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """
        Convert yaw angle to quaternion (rotation around z-axis).
        
        Args:
            yaw: Yaw angle in radians
            
        Returns:
            Quaternion representing rotation
        """
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def calculate_distance_2d(self, point1: Point, point2: Point) -> float:
        """
        Calculate 2D Euclidean distance between two points.
        
        Args:
            point1: First point
            point2: Second point
            
        Returns:
            Distance in meters (ignoring z-axis)
        """
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        return math.sqrt(dx*dx + dy*dy)
    
    def calculate_distance_3d(self, point1: Point, point2: Point) -> float:
        """
        Calculate 3D Euclidean distance between two points.
        
        Args:
            point1: First point
            point2: Second point
            
        Returns:
            Distance in meters
        """
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def calculate_yaw(self, from_point: Point, to_point: Point) -> float:
        """
        Calculate yaw angle from one point to another.
        
        Args:
            from_point: Starting point
            to_point: Target point
            
        Returns:
            Yaw angle in radians (-π to π)
        """
        dx = to_point.x - from_point.x
        dy = to_point.y - from_point.y
        return math.atan2(dy, dx)
    
    def create_point(self, x: float, y: float, z: float) -> Point:
        """
        Create a Point message with specified coordinates.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            z: Z coordinate (altitude) in meters
            
        Returns:
            Point message
        """
        p = Point()
        p.x = x
        p.y = y
        p.z = z
        return p
