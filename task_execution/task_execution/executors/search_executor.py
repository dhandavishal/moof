#!/usr/bin/env python3
"""
Search executor for search and rescue missions.

Generates search patterns optimized for area coverage with sensor FOV.
"""

from typing import Dict, Any, List, Tuple
from geometry_msgs.msg import Point
import math

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

from task_execution.executors.base_executor import BaseExecutor
from multi_drone_msgs.msg import PrimitiveCommand


class SearchExecutor(BaseExecutor):
    """
    Executor for search and rescue tasks.
    
    Generates various search patterns optimized for sensor coverage:
    - Expanding square
    - Sector search
    - Creeping line
    - Spiral
    """
    
    def __init__(self, config: Dict):
        """
        Initialize search executor.
        
        Args:
            config: Configuration dictionary
        """
        super().__init__(config)
        
        if not NUMPY_AVAILABLE:
            self._log_warn("NumPy not available - some search pattern features disabled")
        
        # Get search-specific configuration
        search_config = self.task_config.get('search', {})
        self.default_altitude = search_config.get('altitude', 40.0)
        self.default_spacing = search_config.get('spacing', 30.0)
        
        self.cruise_speed = self.drone_config.get('cruise_speed', 3.0)
        
        self._log_info("Search executor initialized")
    
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitive command sequence for search mission.
        
        Expected parameters:
            - pattern: Search pattern type (expanding_square, sector, creeping_line, spiral)
            - center: Search center point [x, y] or Point
            - altitude: Search altitude in meters
            - radius: Maximum search radius (for expanding/spiral patterns)
            - spacing: Track spacing in meters
            - area: Area bounds for creeping line (min_x, max_x, min_y, max_y)
            - sector_angle: Sector angle in degrees (for sector pattern)
            - velocity: Optional cruise velocity
            - sensor_fov: Optional sensor field of view in degrees
            
        Args:
            parameters: Task parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        pattern = parameters.get('pattern', 'expanding_square')
        center = parameters.get('center', [0, 0])
        altitude = parameters.get('altitude', self.default_altitude)
        radius = parameters.get('radius', 100.0)
        spacing = parameters.get('spacing', self.default_spacing)
        area = parameters.get('area', None)
        sector_angle = parameters.get('sector_angle', 90.0)
        velocity = parameters.get('velocity', self.cruise_speed)
        sensor_fov = parameters.get('sensor_fov', None)
        
        # Adjust spacing based on sensor FOV if provided
        if sensor_fov:
            # Calculate ground FOV width at altitude
            fov_width = 2 * altitude * math.tan(math.radians(sensor_fov / 2))
            # Use 80% of FOV for overlap
            adjusted_spacing = fov_width * 0.8
            if adjusted_spacing != spacing:
                self._log_info(
                    f"Adjusted spacing from {spacing:.1f}m to {adjusted_spacing:.1f}m "
                    f"based on {sensor_fov}° FOV at {altitude}m altitude"
                )
                spacing = adjusted_spacing
        
        # Generate waypoints based on pattern
        if pattern == 'expanding_square':
            waypoints = self._generate_expanding_square(center, altitude, radius, spacing)
        
        elif pattern == 'sector':
            waypoints = self._generate_sector_search(
                center, altitude, radius, sector_angle, spacing
            )
        
        elif pattern == 'creeping_line':
            if not area:
                self._log_error("Creeping line pattern requires 'area' parameter")
                return []
            waypoints = self._generate_creeping_line(area, altitude, spacing)
        
        elif pattern == 'spiral':
            waypoints = self._generate_spiral(center, altitude, radius, spacing)
        
        else:
            self._log_error(f"Unknown search pattern: {pattern}")
            return []
        
        if not waypoints:
            self._log_error(f"Failed to generate waypoints for {pattern} pattern")
            return []
        
        self._log_info(
            f"Generated {len(waypoints)} waypoints for {pattern} search pattern"
        )
        
        # Generate primitive commands
        primitives = []
        for wp in waypoints:
            goto_primitive = self.create_goto_primitive(
                position=wp,
                velocity=velocity,
                acceptance_radius=3.0
            )
            primitives.append(goto_primitive)
        
        self._log_info(f"Generated {len(primitives)} primitive commands for search")
        return primitives
    
    def _generate_expanding_square(self,
                                   center: List[float],
                                   altitude: float,
                                   max_radius: float,
                                   spacing: float) -> List[Point]:
        """
        Generate expanding square search pattern.
        
        Starts at center and spirals outward in square pattern.
        
        Args:
            center: Center point [x, y]
            altitude: Search altitude
            max_radius: Maximum distance from center
            spacing: Distance between legs
            
        Returns:
            List of waypoint Points
        """
        waypoints = []
        
        # Start at center
        cx, cy = center[0], center[1]
        x, y = cx, cy
        
        waypoints.append(Point(x=x, y=y, z=altitude))
        
        # Expanding square spiral
        leg_length = spacing
        direction = 0  # 0=East, 1=North, 2=West, 3=South
        
        while leg_length < max_radius * 2:
            # Move in current direction
            if direction == 0:  # East
                x += leg_length
            elif direction == 1:  # North
                y += leg_length
            elif direction == 2:  # West
                x -= leg_length
            elif direction == 3:  # South
                y -= leg_length
            
            waypoints.append(Point(x=x, y=y, z=altitude))
            
            # Change direction
            direction = (direction + 1) % 4
            
            # Increase leg length after every two legs
            if direction % 2 == 0:
                leg_length += spacing
        
        self._log_debug(f"Expanding square: {len(waypoints)} waypoints, max leg {leg_length:.1f}m")
        return waypoints
    
    def _generate_sector_search(self,
                                center: List[float],
                                altitude: float,
                                radius: float,
                                sector_angle: float,
                                spacing: float) -> List[Point]:
        """
        Generate sector search pattern.
        
        Sweeps back and forth across a sector, expanding outward.
        
        Args:
            center: Center point [x, y]
            altitude: Search altitude
            radius: Search radius
            sector_angle: Sector angle in degrees
            spacing: Radial spacing
            
        Returns:
            List of waypoint Points
        """
        waypoints = []
        
        cx, cy = center[0], center[1]
        
        # Start at center
        waypoints.append(Point(x=cx, y=cy, z=altitude))
        
        # Number of radial segments
        num_segments = int(radius / spacing)
        
        # Sweep angle range
        start_angle = -sector_angle / 2
        end_angle = sector_angle / 2
        
        # Alternating sweeps at increasing radii
        for i in range(1, num_segments + 1):
            r = i * spacing
            
            if i % 2 == 1:
                # Sweep left to right
                angle1 = math.radians(start_angle)
                angle2 = math.radians(end_angle)
            else:
                # Sweep right to left
                angle1 = math.radians(end_angle)
                angle2 = math.radians(start_angle)
            
            x1 = cx + r * math.cos(angle1)
            y1 = cy + r * math.sin(angle1)
            
            x2 = cx + r * math.cos(angle2)
            y2 = cy + r * math.sin(angle2)
            
            waypoints.append(Point(x=x1, y=y1, z=altitude))
            waypoints.append(Point(x=x2, y=y2, z=altitude))
        
        self._log_debug(
            f"Sector search: {len(waypoints)} waypoints, "
            f"{sector_angle}° sector, {num_segments} radial segments"
        )
        return waypoints
    
    def _generate_creeping_line(self,
                                area: Tuple[float, float, float, float],
                                altitude: float,
                                spacing: float) -> List[Point]:
        """
        Generate creeping line search pattern.
        
        Parallel tracks across the search area (like lawn-mower).
        
        Args:
            area: (min_x, max_x, min_y, max_y) bounds
            altitude: Search altitude
            spacing: Track spacing
            
        Returns:
            List of waypoint Points
        """
        min_x, max_x, min_y, max_y = area
        
        waypoints = []
        
        y = min_y
        direction = 1  # 1 for left-to-right, -1 for right-to-left
        
        while y <= max_y:
            if direction == 1:
                waypoints.append(Point(x=min_x, y=y, z=altitude))
                waypoints.append(Point(x=max_x, y=y, z=altitude))
            else:
                waypoints.append(Point(x=max_x, y=y, z=altitude))
                waypoints.append(Point(x=min_x, y=y, z=altitude))
            
            y += spacing
            direction *= -1
        
        self._log_debug(
            f"Creeping line: {len(waypoints)} waypoints, "
            f"{int((max_y - min_y) / spacing) + 1} tracks"
        )
        return waypoints
    
    def _generate_spiral(self,
                        center: List[float],
                        altitude: float,
                        max_radius: float,
                        spacing: float) -> List[Point]:
        """
        Generate spiral search pattern.
        
        Archimedean spiral expanding from center.
        
        Args:
            center: Center point [x, y]
            altitude: Search altitude
            max_radius: Maximum spiral radius
            spacing: Distance between spiral arms
            
        Returns:
            List of waypoint Points
        """
        if not NUMPY_AVAILABLE:
            self._log_warn("NumPy not available - using simplified spiral")
            # Fallback to expanding square
            return self._generate_expanding_square(center, altitude, max_radius, spacing)
        
        waypoints = []
        
        cx, cy = center[0], center[1]
        
        # Archimedean spiral: r = a * theta
        # where a = spacing / (2 * pi) to get desired spacing between arms
        a = spacing / (2 * math.pi)
        
        # Calculate theta range
        max_theta = max_radius / a
        
        # Generate spiral points
        # Use smaller theta steps for smoother spiral
        theta_step = 0.2  # radians
        theta = 0
        
        while theta <= max_theta:
            r = a * theta
            
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)
            
            waypoints.append(Point(x=x, y=y, z=altitude))
            
            theta += theta_step
        
        self._log_debug(
            f"Spiral: {len(waypoints)} waypoints, "
            f"{max_theta:.1f} radians, {spacing:.1f}m spacing"
        )
        return waypoints
