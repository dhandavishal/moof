#!/usr/bin/env python3
"""
Waypoint navigation executor.

Executes a sequence of waypoint navigation commands with optional actions.
"""

from typing import Dict, Any, List
from geometry_msgs.msg import Point
import math

from task_execution.executors.base_executor import BaseExecutor
from multi_drone_msgs.msg import PrimitiveCommand


class WaypointExecutor(BaseExecutor):
    """
    Executor for waypoint navigation tasks.
    
    Handles simple point-to-point navigation with optional actions at waypoints
    (hover, photo, scan, etc.).
    """
    
    def __init__(self, config: Dict):
        """
        Initialize waypoint executor.
        
        Args:
            config: Configuration dictionary
        """
        super().__init__(config)
        
        # Get waypoint-specific configuration
        wp_config = self.task_config.get('waypoint', {})
        self.default_acceptance_radius = wp_config.get('acceptance_radius', 1.0)
        self.max_waypoint_distance = wp_config.get('max_waypoint_distance', 5000.0)
        
        self.cruise_speed = self.drone_config.get('cruise_speed', 2.0)
        
        self._log_info("Waypoint executor initialized")
    
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitive command sequence for waypoint navigation.
        
        Expected parameters:
            - waypoints: List of waypoint dictionaries with keys:
                - position: [x, y, z] or Point
                - action: Optional action at waypoint (hover, photo, etc.)
                - dwell_time: Optional time to hover at waypoint
                - yaw: Optional yaw angle in degrees
            - velocity: Optional cruise velocity
            - loop: Optional boolean to return to start
            
        Args:
            parameters: Task parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        waypoints = parameters.get('waypoints', [])
        velocity = parameters.get('velocity', self.cruise_speed)
        loop = parameters.get('loop', False)
        
        if not waypoints:
            self._log_error("Waypoint executor received empty waypoint list")
            return []
        
        self._log_info(f"Generating primitive commands for {len(waypoints)} waypoints")
        
        primitives = []
        
        # Process each waypoint
        for i, wp in enumerate(waypoints):
            # Extract position
            if isinstance(wp, dict):
                position = self._extract_position(wp.get('position'))
                action = wp.get('action', None)
                dwell_time = wp.get('dwell_time', 0.0)
                yaw = wp.get('yaw', None)
            else:
                # Assume wp is directly a position
                position = self._extract_position(wp)
                action = None
                dwell_time = 0.0
                yaw = None
            
            # Validate waypoint distance
            if i > 0:
                prev_pos = primitives[-1].target_position if primitives else position
                distance = self.calculate_distance_3d(prev_pos, position)
                
                if distance > self.max_waypoint_distance:
                    self._log_warn(
                        f"Waypoint {i} is {distance:.1f}m from previous "
                        f"(max {self.max_waypoint_distance}m)"
                    )
            
            # Convert yaw from degrees to radians if provided
            yaw_rad = None
            if yaw is not None:
                yaw_rad = math.radians(yaw)
            
            # Create goto primitive command
            goto_primitive = self.create_goto_primitive(
                position=position,
                velocity=velocity,
                acceptance_radius=self.default_acceptance_radius,
                yaw=yaw_rad
            )
            primitives.append(goto_primitive)
            
            self._log_debug(
                f"Waypoint {i+1}: goto ({position.x:.1f}, {position.y:.1f}, {position.z:.1f})"
            )
            
            # Add action if specified
            if action or dwell_time > 0:
                action_primitives = self._handle_waypoint_action(
                    action, position, dwell_time
                )
                primitives.extend(action_primitives)
        
        # Add loop back to start if requested
        if loop and len(waypoints) > 1:
            first_position = self._extract_position(
                waypoints[0]['position'] if isinstance(waypoints[0], dict) else waypoints[0]
            )
            loop_primitive = self.create_goto_primitive(
                position=first_position,
                velocity=velocity
            )
            primitives.append(loop_primitive)
            self._log_info("Added loop back to start position")
        
        self._log_info(f"Generated {len(primitives)} primitive commands for waypoint task")
        return primitives
    
    def _extract_position(self, position_data) -> Point:
        """
        Convert various position formats to Point.
        
        Args:
            position_data: Position as Point, list [x,y,z], or dict
            
        Returns:
            Point object
        """
        if isinstance(position_data, Point):
            return position_data
        
        elif isinstance(position_data, (list, tuple)):
            point = Point()
            point.x = float(position_data[0])
            point.y = float(position_data[1])
            point.z = float(position_data[2]) if len(position_data) > 2 else 0.0
            return point
        
        elif isinstance(position_data, dict):
            point = Point()
            point.x = float(position_data.get('x', 0.0))
            point.y = float(position_data.get('y', 0.0))
            point.z = float(position_data.get('z', 0.0))
            return point
        
        else:
            self._log_error(f"Unknown position format: {type(position_data)}")
            return Point()
    
    def _handle_waypoint_action(self, 
                                action: str, 
                                position: Point,
                                dwell_time: float) -> List[PrimitiveCommand]:
        """
        Generate primitive commands for waypoint actions.
        
        Args:
            action: Action type (hover, photo, scan, etc.)
            position: Current waypoint position
            dwell_time: Time to dwell at waypoint
            
        Returns:
            List of action primitive commands
        """
        primitives = []
        
        if action == 'hover' or dwell_time > 0:
            # Add loiter primitive
            loiter_duration = dwell_time if dwell_time > 0 else 5.0
            loiter = self.create_loiter_primitive(
                position=position,
                duration=loiter_duration,
                radius=2.0
            )
            primitives.append(loiter)
            self._log_debug(f"Added hover action for {loiter_duration}s")
        
        elif action == 'photo':
            # Add brief hover for photo stabilization
            loiter = self.create_loiter_primitive(
                position=position,
                duration=2.0,
                radius=1.0
            )
            primitives.append(loiter)
            self._log_debug("Added photo action (2s stabilization)")
            
            # Note: Actual camera trigger would be handled by payload controller
        
        elif action == 'scan':
            # Rotate 360 degrees at position
            loiter = self.create_loiter_primitive(
                position=position,
                duration=15.0,  # Time for 360° rotation
                radius=2.0
            )
            primitives.append(loiter)
            self._log_debug("Added scan action (15s for 360° rotation)")
        
        elif action:
            self._log_warn(f"Unknown waypoint action: {action}")
        
        return primitives
