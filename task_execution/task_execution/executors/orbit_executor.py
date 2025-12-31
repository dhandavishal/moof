#!/usr/bin/env python3
"""
Orbit executor for circular flight patterns.

Executes circular orbit/loiter patterns around a specified center point.
Supports both single orbits and continuous orbiting with configurable parameters.
"""

from typing import Dict, Any, List
from geometry_msgs.msg import Point
import math

from task_execution.executors.base_executor import BaseExecutor
from multi_drone_msgs.msg import PrimitiveCommand


class OrbitExecutor(BaseExecutor):
    """
    Executor for orbit/circle flight patterns.
    
    Generates waypoints around a center point to create circular flight paths.
    Supports:
    - Clockwise and counter-clockwise orbits
    - Variable radius and altitude
    - Configurable number of orbit segments (smoothness)
    - Multiple orbits (laps)
    - Face-center or tangential heading modes
    
    Example mission parameters:
    {
        "center": {"x": 50.0, "y": 50.0, "z": 30.0},
        "radius": 25.0,
        "num_points": 12,
        "num_orbits": 2,
        "clockwise": true,
        "face_center": true,
        "velocity": 3.0,
        "entry_angle": 0.0
    }
    """
    
    # Heading modes
    HEADING_FACE_CENTER = "face_center"     # Always face the center point
    HEADING_TANGENTIAL = "tangential"       # Face direction of travel
    HEADING_FIXED = "fixed"                 # Maintain initial heading
    
    def __init__(self, config: Dict):
        """
        Initialize orbit executor.
        
        Args:
            config: Configuration dictionary
        """
        super().__init__(config)
        
        # Get orbit-specific configuration with defaults
        orbit_config = self.task_config.get('orbit', {})
        self.default_radius = orbit_config.get('default_radius', 20.0)
        self.default_num_points = orbit_config.get('default_num_points', 12)
        self.default_num_orbits = orbit_config.get('default_num_orbits', 1)
        self.min_radius = orbit_config.get('min_radius', 5.0)
        self.max_radius = orbit_config.get('max_radius', 200.0)
        
        self.cruise_speed = self.drone_config.get('cruise_speed', 2.0)
        
        self._log_info("Orbit executor initialized")
    
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitive command sequence for orbit pattern.
        
        Expected parameters:
            - center: Center point {"x": float, "y": float, "z": float} or altitude only
            - radius: Orbit radius in meters
            - num_points: Number of waypoints per orbit (smoothness)
            - num_orbits: Number of complete orbits (laps)
            - clockwise: Direction of orbit (default: True)
            - face_center: If True, drone faces center; if False, faces tangent
            - velocity: Orbit velocity in m/s
            - entry_angle: Starting angle in degrees (0 = East, 90 = North)
            - altitude: Orbit altitude (overrides center.z if provided)
            
        Args:
            parameters: Task parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        # Extract and validate parameters
        center = self._extract_center(parameters)
        radius = parameters.get('radius', self.default_radius)
        num_points = parameters.get('num_points', self.default_num_points)
        num_orbits = parameters.get('num_orbits', self.default_num_orbits)
        clockwise = parameters.get('clockwise', True)
        face_center = parameters.get('face_center', True)
        velocity = parameters.get('velocity', self.cruise_speed)
        entry_angle_deg = parameters.get('entry_angle', 0.0)
        
        # Override altitude if specified separately
        if 'altitude' in parameters:
            center.z = parameters['altitude']
        
        # Validate radius
        if radius < self.min_radius:
            self._log_warn(f"Radius {radius}m below minimum {self.min_radius}m, clamping")
            radius = self.min_radius
        elif radius > self.max_radius:
            self._log_warn(f"Radius {radius}m above maximum {self.max_radius}m, clamping")
            radius = self.max_radius
        
        # Validate num_points (minimum 4 for a reasonable circle)
        if num_points < 4:
            self._log_warn(f"num_points {num_points} too low, using 4")
            num_points = 4
        elif num_points > 36:
            self._log_warn(f"num_points {num_points} too high, capping at 36")
            num_points = 36
        
        self._log_info(
            f"Generating orbit: center=({center.x:.1f}, {center.y:.1f}, {center.z:.1f}), "
            f"radius={radius:.1f}m, points={num_points}, orbits={num_orbits}, "
            f"{'CW' if clockwise else 'CCW'}"
        )
        
        primitives = []
        
        # Convert entry angle to radians
        entry_angle_rad = math.radians(entry_angle_deg)
        
        # Direction multiplier: +1 for CCW (standard math), -1 for CW
        direction = -1 if clockwise else 1
        
        # Calculate total waypoints for all orbits
        # +1 to close back to start on final orbit
        total_points = num_points * num_orbits + 1
        
        # Generate orbit waypoints
        for i in range(total_points):
            # Calculate angle for this point
            # Progress around circle: 0 to 2*pi per orbit
            angle = entry_angle_rad + direction * (2 * math.pi * i / num_points)
            
            # Calculate position on circle
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)
            z = center.z
            
            position = self.create_point(x, y, z)
            
            # Calculate yaw based on heading mode
            if face_center:
                # Face toward center
                yaw = self.calculate_yaw(position, center)
            else:
                # Tangential: face direction of travel
                # Tangent is perpendicular to radius
                if clockwise:
                    yaw = angle - math.pi / 2  # 90 degrees behind radius
                else:
                    yaw = angle + math.pi / 2  # 90 degrees ahead of radius
            
            # Normalize yaw to -pi to pi
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            
            # Create goto primitive
            goto_cmd = self.create_goto_primitive(
                position=position,
                velocity=velocity,
                acceptance_radius=max(radius * 0.1, 1.0),  # 10% of radius, min 1m
                yaw=yaw,
                timeout=120.0
            )
            primitives.append(goto_cmd)
            
            self._log_debug(
                f"Orbit point {i+1}/{total_points}: "
                f"({x:.1f}, {y:.1f}, {z:.1f}), yaw={math.degrees(yaw):.1f}°"
            )
        
        self._log_info(f"Generated {len(primitives)} primitives for orbit task")
        return primitives
    
    def _extract_center(self, parameters: Dict[str, Any]) -> Point:
        """
        Extract center point from parameters.
        
        Handles multiple input formats:
        - {"center": {"x": 10, "y": 20, "z": 30}}
        - {"center": [10, 20, 30]}
        - {"center_x": 10, "center_y": 20, "altitude": 30}
        
        Args:
            parameters: Task parameters
            
        Returns:
            Point representing orbit center
        """
        center = parameters.get('center', {})
        
        if isinstance(center, dict):
            x = center.get('x', 0.0)
            y = center.get('y', 0.0)
            z = center.get('z', self.drone_config.get('default_altitude', 30.0))
        elif isinstance(center, (list, tuple)) and len(center) >= 2:
            x = center[0]
            y = center[1]
            z = center[2] if len(center) > 2 else self.drone_config.get('default_altitude', 30.0)
        else:
            # Fallback: try individual parameters
            x = parameters.get('center_x', 0.0)
            y = parameters.get('center_y', 0.0)
            z = parameters.get('altitude', self.drone_config.get('default_altitude', 30.0))
        
        return self.create_point(float(x), float(y), float(z))
    
    def generate_orbit_with_entry_exit(
        self,
        parameters: Dict[str, Any]
    ) -> List[PrimitiveCommand]:
        """
        Generate orbit with smooth entry and exit paths.
        
        Creates a flight path that:
        1. Flies to orbit entry point
        2. Performs circular orbit(s)
        3. Exits orbit and optionally returns to start
        
        Additional parameters:
            - start_position: Starting position {"x", "y", "z"}
            - return_to_start: Return to start after orbit (default: False)
            
        Args:
            parameters: Task parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        primitives = []
        
        start_pos = parameters.get('start_position')
        return_to_start = parameters.get('return_to_start', False)
        
        center = self._extract_center(parameters)
        radius = parameters.get('radius', self.default_radius)
        entry_angle_deg = parameters.get('entry_angle', 0.0)
        velocity = parameters.get('velocity', self.cruise_speed)
        
        # Calculate entry point
        entry_angle_rad = math.radians(entry_angle_deg)
        entry_x = center.x + radius * math.cos(entry_angle_rad)
        entry_y = center.y + radius * math.sin(entry_angle_rad)
        entry_point = self.create_point(entry_x, entry_y, center.z)
        
        # If start position provided, add transit to entry point
        if start_pos:
            start = self._extract_position(start_pos)
            
            # Calculate yaw toward entry point
            entry_yaw = self.calculate_yaw(start, entry_point)
            
            transit_cmd = self.create_goto_primitive(
                position=entry_point,
                velocity=velocity,
                yaw=entry_yaw,
                timeout=180.0
            )
            primitives.append(transit_cmd)
            self._log_info("Added transit to orbit entry point")
        
        # Generate main orbit
        orbit_primitives = self.execute(parameters)
        primitives.extend(orbit_primitives)
        
        # Return to start if requested
        if return_to_start and start_pos:
            start = self._extract_position(start_pos)
            exit_point = primitives[-1].target_position if primitives else entry_point
            
            return_yaw = self.calculate_yaw(exit_point, start)
            return_cmd = self.create_goto_primitive(
                position=start,
                velocity=velocity,
                yaw=return_yaw,
                timeout=180.0
            )
            primitives.append(return_cmd)
            self._log_info("Added return to start position")
        
        return primitives
    
    def _extract_position(self, pos_data) -> Point:
        """Extract Point from various position formats."""
        if isinstance(pos_data, Point):
            return pos_data
        elif isinstance(pos_data, dict):
            return self.create_point(
                pos_data.get('x', 0.0),
                pos_data.get('y', 0.0),
                pos_data.get('z', 30.0)
            )
        elif isinstance(pos_data, (list, tuple)):
            return self.create_point(
                pos_data[0] if len(pos_data) > 0 else 0.0,
                pos_data[1] if len(pos_data) > 1 else 0.0,
                pos_data[2] if len(pos_data) > 2 else 30.0
            )
        return self.create_point(0.0, 0.0, 30.0)


class SpiralOrbitExecutor(OrbitExecutor):
    """
    Extension of OrbitExecutor for spiral patterns.
    
    Creates expanding or contracting spiral patterns for area coverage
    or dramatic camera movements.
    
    Additional parameters:
        - start_radius: Initial radius
        - end_radius: Final radius
        - altitude_change: Change in altitude over spiral
    """
    
    def __init__(self, config: Dict):
        super().__init__(config)
        self._log_info("Spiral orbit executor initialized")
    
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """Generate spiral pattern."""
        center = self._extract_center(parameters)
        start_radius = parameters.get('start_radius', 10.0)
        end_radius = parameters.get('end_radius', 50.0)
        num_points = parameters.get('num_points', 24)
        num_orbits = parameters.get('num_orbits', 2)
        clockwise = parameters.get('clockwise', True)
        face_center = parameters.get('face_center', True)
        velocity = parameters.get('velocity', self.cruise_speed)
        altitude_change = parameters.get('altitude_change', 0.0)
        
        start_altitude = center.z
        
        self._log_info(
            f"Generating spiral: radius {start_radius:.1f}m → {end_radius:.1f}m, "
            f"{num_orbits} orbits, {num_points} points/orbit"
        )
        
        primitives = []
        total_points = num_points * num_orbits
        direction = -1 if clockwise else 1
        
        for i in range(total_points + 1):
            # Progress through spiral (0.0 to 1.0)
            progress = i / total_points
            
            # Interpolate radius
            current_radius = start_radius + (end_radius - start_radius) * progress
            
            # Interpolate altitude
            current_altitude = start_altitude + altitude_change * progress
            
            # Calculate angle
            angle = direction * (2 * math.pi * i / num_points)
            
            # Calculate position
            x = center.x + current_radius * math.cos(angle)
            y = center.y + current_radius * math.sin(angle)
            z = current_altitude
            
            position = self.create_point(x, y, z)
            
            # Calculate yaw
            if face_center:
                yaw = self.calculate_yaw(position, self.create_point(center.x, center.y, z))
            else:
                yaw = angle + (math.pi / 2 * direction)
            
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            
            goto_cmd = self.create_goto_primitive(
                position=position,
                velocity=velocity,
                acceptance_radius=max(current_radius * 0.1, 1.0),
                yaw=yaw
            )
            primitives.append(goto_cmd)
        
        self._log_info(f"Generated {len(primitives)} primitives for spiral pattern")
        return primitives
