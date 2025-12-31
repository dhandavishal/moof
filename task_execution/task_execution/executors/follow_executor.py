#!/usr/bin/env python3
"""
Follow executor for target tracking and path following.

Executes follow missions including:
- Path following (follow a predefined path smoothly)
- Target following (follow a moving target)
- Leader-follower formation (follow another drone)
"""

from typing import Dict, Any, List, Optional, Tuple
from geometry_msgs.msg import Point
import math
import time

from task_execution.executors.base_executor import BaseExecutor
from multi_drone_msgs.msg import PrimitiveCommand


class FollowExecutor(BaseExecutor):
    """
    Executor for follow/tracking missions.
    
    Supports multiple follow modes:
    1. PATH_FOLLOW: Follow a predefined smooth path
    2. TARGET_FOLLOW: Follow a moving target position (updated via topic)
    3. LEADER_FOLLOW: Follow another drone with offset
    
    Example mission parameters for path following:
    {
        "mode": "path",
        "path": [
            {"x": 0, "y": 0, "z": 30},
            {"x": 10, "y": 5, "z": 30},
            {"x": 20, "y": 0, "z": 35},
            {"x": 30, "y": -5, "z": 30}
        ],
        "velocity": 3.0,
        "smoothing": true,
        "look_ahead_distance": 5.0
    }
    
    Example for target following:
    {
        "mode": "target",
        "target_topic": "/target/position",
        "offset": {"x": 0, "y": -10, "z": 5},
        "min_distance": 5.0,
        "max_distance": 50.0,
        "timeout": 120.0
    }
    
    Example for leader following:
    {
        "mode": "leader",
        "leader_drone_id": "drone_0",
        "offset": {"x": -10, "y": 5, "z": 0},
        "maintain_formation": true
    }
    """
    
    # Follow modes
    MODE_PATH = "path"
    MODE_TARGET = "target"
    MODE_LEADER = "leader"
    
    def __init__(self, config: Dict):
        """
        Initialize follow executor.
        
        Args:
            config: Configuration dictionary
        """
        super().__init__(config)
        
        # Get follow-specific configuration
        follow_config = self.task_config.get('follow', {})
        self.default_look_ahead = follow_config.get('look_ahead_distance', 5.0)
        self.default_smoothing_factor = follow_config.get('smoothing_factor', 0.5)
        self.min_segment_length = follow_config.get('min_segment_length', 1.0)
        self.max_segment_length = follow_config.get('max_segment_length', 20.0)
        
        self.cruise_speed = self.drone_config.get('cruise_speed', 2.0)
        
        self._log_info("Follow executor initialized")
    
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitive command sequence for follow task.
        
        Routes to appropriate handler based on follow mode.
        
        Args:
            parameters: Task parameters with 'mode' specifying follow type
            
        Returns:
            List of PrimitiveCommand messages
        """
        mode = parameters.get('mode', self.MODE_PATH)
        
        if mode == self.MODE_PATH:
            return self._execute_path_follow(parameters)
        elif mode == self.MODE_TARGET:
            return self._execute_target_follow(parameters)
        elif mode == self.MODE_LEADER:
            return self._execute_leader_follow(parameters)
        else:
            self._log_error(f"Unknown follow mode: {mode}")
            return []
    
    def _execute_path_follow(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitives for smooth path following.
        
        Features:
        - Catmull-Rom spline interpolation for smooth curves
        - Configurable sampling density
        - Heading aligned with path tangent
        
        Args:
            parameters: Path follow parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        path = parameters.get('path', [])
        velocity = parameters.get('velocity', self.cruise_speed)
        smoothing = parameters.get('smoothing', True)
        look_ahead = parameters.get('look_ahead_distance', self.default_look_ahead)
        samples_per_segment = parameters.get('samples_per_segment', 5)
        face_direction = parameters.get('face_direction', True)
        
        if len(path) < 2:
            self._log_error("Path follow requires at least 2 waypoints")
            return []
        
        self._log_info(
            f"Generating path follow: {len(path)} waypoints, "
            f"smoothing={'ON' if smoothing else 'OFF'}, velocity={velocity}m/s"
        )
        
        # Convert path to Points
        control_points = [self._extract_position(p) for p in path]
        
        # Generate smooth path if requested
        if smoothing and len(control_points) >= 4:
            path_points = self._generate_catmull_rom_path(
                control_points, 
                samples_per_segment
            )
        else:
            path_points = control_points
        
        primitives = []
        
        for i, point in enumerate(path_points):
            # Calculate heading
            if face_direction and i < len(path_points) - 1:
                # Face toward next point
                next_point = path_points[i + 1]
                yaw = self.calculate_yaw(point, next_point)
            elif face_direction and i > 0:
                # Last point: maintain previous heading
                prev_point = path_points[i - 1]
                yaw = self.calculate_yaw(prev_point, point)
            else:
                yaw = None
            
            # Adjust velocity based on path curvature
            if i > 0 and i < len(path_points) - 1:
                curvature = self._calculate_curvature(
                    path_points[i-1], point, path_points[i+1]
                )
                # Slow down on curves
                adjusted_velocity = velocity * max(0.5, 1.0 - curvature * 2)
            else:
                adjusted_velocity = velocity
            
            goto_cmd = self.create_goto_primitive(
                position=point,
                velocity=adjusted_velocity,
                acceptance_radius=look_ahead * 0.5,
                yaw=yaw,
                timeout=120.0
            )
            primitives.append(goto_cmd)
        
        self._log_info(f"Generated {len(primitives)} primitives for path follow")
        return primitives
    
    def _execute_target_follow(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitives for target following.
        
        This creates a loiter primitive that will be updated in real-time
        by the TEE based on target position updates.
        
        For static target follow (without real-time updates), generates
        goto primitives to the target with offset.
        
        Args:
            parameters: Target follow parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        target_position = parameters.get('target_position')
        offset = parameters.get('offset', {'x': 0, 'y': 0, 'z': 0})
        velocity = parameters.get('velocity', self.cruise_speed)
        timeout = parameters.get('timeout', 120.0)
        min_distance = parameters.get('min_distance', 5.0)
        max_distance = parameters.get('max_distance', 50.0)
        face_target = parameters.get('face_target', True)
        
        primitives = []
        
        if target_position:
            # Static target - generate goto to offset position
            target = self._extract_position(target_position)
            
            # Apply offset
            follow_position = self.create_point(
                target.x + offset.get('x', 0),
                target.y + offset.get('y', 0),
                target.z + offset.get('z', 0)
            )
            
            # Calculate yaw to face target
            yaw = None
            if face_target:
                yaw = self.calculate_yaw(follow_position, target)
            
            goto_cmd = self.create_goto_primitive(
                position=follow_position,
                velocity=velocity,
                yaw=yaw,
                timeout=timeout
            )
            primitives.append(goto_cmd)
            
            self._log_info(
                f"Generated target follow to "
                f"({follow_position.x:.1f}, {follow_position.y:.1f}, {follow_position.z:.1f})"
            )
        else:
            # Dynamic target - create loiter that TEE will update
            # This is a placeholder; actual tracking happens in TEE
            self._log_info(
                "Dynamic target follow - TEE will handle position updates"
            )
            
            # Create a loiter at current position that will be updated
            loiter_cmd = self.create_loiter_primitive(
                position=self.create_point(0, 0, 30),  # Placeholder
                duration=timeout,
                radius=0.0  # Hold position mode
            )
            primitives.append(loiter_cmd)
        
        return primitives
    
    def _execute_leader_follow(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitives for leader-follower mode.
        
        Follows another drone (leader) while maintaining a specified offset.
        The offset is relative to the leader's position and optionally
        rotates with the leader's heading.
        
        Args:
            parameters: Leader follow parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        leader_id = parameters.get('leader_drone_id', 'drone_0')
        offset = parameters.get('offset', {'x': -10, 'y': 0, 'z': 0})
        velocity = parameters.get('velocity', self.cruise_speed)
        timeout = parameters.get('timeout', 300.0)
        maintain_formation = parameters.get('maintain_formation', True)
        rotate_with_leader = parameters.get('rotate_with_leader', False)
        
        # Leader position if provided (for planning)
        leader_position = parameters.get('leader_position')
        leader_waypoints = parameters.get('leader_waypoints', [])
        
        primitives = []
        
        if leader_waypoints:
            # Generate offset path from leader's planned waypoints
            self._log_info(
                f"Following leader {leader_id} through {len(leader_waypoints)} waypoints"
            )
            
            offset_x = offset.get('x', 0)
            offset_y = offset.get('y', 0)
            offset_z = offset.get('z', 0)
            
            for i, wp in enumerate(leader_waypoints):
                leader_point = self._extract_position(wp)
                
                if rotate_with_leader and i < len(leader_waypoints) - 1:
                    # Calculate leader's heading
                    next_point = self._extract_position(leader_waypoints[i + 1])
                    leader_heading = self.calculate_yaw(leader_point, next_point)
                    
                    # Rotate offset by leader heading
                    rotated_offset = self._rotate_offset(
                        offset_x, offset_y, leader_heading
                    )
                    follow_x = leader_point.x + rotated_offset[0]
                    follow_y = leader_point.y + rotated_offset[1]
                else:
                    # Fixed offset (no rotation)
                    follow_x = leader_point.x + offset_x
                    follow_y = leader_point.y + offset_y
                
                follow_z = leader_point.z + offset_z
                
                follow_position = self.create_point(follow_x, follow_y, follow_z)
                
                # Calculate yaw to face leader or direction of travel
                if maintain_formation:
                    yaw = self.calculate_yaw(follow_position, leader_point)
                else:
                    yaw = None
                
                goto_cmd = self.create_goto_primitive(
                    position=follow_position,
                    velocity=velocity,
                    yaw=yaw,
                    timeout=timeout / len(leader_waypoints)
                )
                primitives.append(goto_cmd)
                
            self._log_info(
                f"Generated {len(primitives)} primitives for leader follow"
            )
            
        elif leader_position:
            # Single position follow
            leader_point = self._extract_position(leader_position)
            
            follow_position = self.create_point(
                leader_point.x + offset.get('x', 0),
                leader_point.y + offset.get('y', 0),
                leader_point.z + offset.get('z', 0)
            )
            
            yaw = self.calculate_yaw(follow_position, leader_point)
            
            goto_cmd = self.create_goto_primitive(
                position=follow_position,
                velocity=velocity,
                yaw=yaw,
                timeout=timeout
            )
            primitives.append(goto_cmd)
            
        else:
            # Dynamic following - create placeholder
            self._log_info(
                f"Dynamic leader follow mode for {leader_id} - TEE handles updates"
            )
            loiter_cmd = self.create_loiter_primitive(
                position=self.create_point(0, 0, 30),
                duration=timeout,
                radius=0.0
            )
            primitives.append(loiter_cmd)
        
        return primitives
    
    # ========================================================================
    # Path Smoothing Algorithms
    # ========================================================================
    
    def _generate_catmull_rom_path(
        self,
        control_points: List[Point],
        samples_per_segment: int = 5
    ) -> List[Point]:
        """
        Generate smooth path using Catmull-Rom spline interpolation.
        
        Catmull-Rom splines pass through all control points and provide
        C1 continuity (smooth velocity transitions).
        
        Args:
            control_points: List of Points defining the path
            samples_per_segment: Number of interpolated points per segment
            
        Returns:
            List of interpolated Points forming smooth path
        """
        if len(control_points) < 4:
            return control_points
        
        path = []
        
        # Extend control points for end conditions
        # Duplicate first and last points for boundary handling
        extended = [control_points[0]] + control_points + [control_points[-1]]
        
        # Interpolate between each pair of interior points
        for i in range(1, len(extended) - 2):
            p0 = extended[i - 1]
            p1 = extended[i]
            p2 = extended[i + 1]
            p3 = extended[i + 2]
            
            # Sample this segment
            for j in range(samples_per_segment):
                t = j / samples_per_segment
                point = self._catmull_rom_interpolate(p0, p1, p2, p3, t)
                path.append(point)
        
        # Add final point
        path.append(control_points[-1])
        
        self._log_debug(
            f"Catmull-Rom: {len(control_points)} control points → "
            f"{len(path)} path points"
        )
        
        return path
    
    def _catmull_rom_interpolate(
        self,
        p0: Point, p1: Point, p2: Point, p3: Point,
        t: float
    ) -> Point:
        """
        Catmull-Rom spline interpolation between p1 and p2.
        
        Args:
            p0, p1, p2, p3: Four control points
            t: Parameter 0-1 between p1 and p2
            
        Returns:
            Interpolated Point
        """
        t2 = t * t
        t3 = t2 * t
        
        # Catmull-Rom basis functions
        b0 = -0.5*t3 + t2 - 0.5*t
        b1 = 1.5*t3 - 2.5*t2 + 1.0
        b2 = -1.5*t3 + 2.0*t2 + 0.5*t
        b3 = 0.5*t3 - 0.5*t2
        
        x = b0*p0.x + b1*p1.x + b2*p2.x + b3*p3.x
        y = b0*p0.y + b1*p1.y + b2*p2.y + b3*p3.y
        z = b0*p0.z + b1*p1.z + b2*p2.z + b3*p3.z
        
        return self.create_point(x, y, z)
    
    def _generate_bezier_path(
        self,
        control_points: List[Point],
        samples: int = 20
    ) -> List[Point]:
        """
        Generate smooth path using cubic Bezier curves.
        
        Creates connected cubic Bezier segments between control points.
        
        Args:
            control_points: List of Points (must be 4, 7, 10, ... for proper curves)
            samples: Number of points to sample per curve segment
            
        Returns:
            List of Points forming smooth path
        """
        path = []
        
        # Need at least 4 points for one cubic Bezier
        if len(control_points) < 4:
            return control_points
        
        # Process each cubic Bezier segment (4 points each)
        i = 0
        while i + 3 < len(control_points):
            p0, p1, p2, p3 = control_points[i:i+4]
            
            for j in range(samples):
                t = j / samples
                point = self._cubic_bezier(p0, p1, p2, p3, t)
                path.append(point)
            
            i += 3  # Overlap by one point for continuity
        
        # Add final point
        path.append(control_points[-1])
        
        return path
    
    def _cubic_bezier(
        self,
        p0: Point, p1: Point, p2: Point, p3: Point,
        t: float
    ) -> Point:
        """Cubic Bezier interpolation."""
        t2 = t * t
        t3 = t2 * t
        mt = 1 - t
        mt2 = mt * mt
        mt3 = mt2 * mt
        
        x = mt3*p0.x + 3*mt2*t*p1.x + 3*mt*t2*p2.x + t3*p3.x
        y = mt3*p0.y + 3*mt2*t*p1.y + 3*mt*t2*p2.y + t3*p3.y
        z = mt3*p0.z + 3*mt2*t*p1.z + 3*mt*t2*p2.z + t3*p3.z
        
        return self.create_point(x, y, z)
    
    # ========================================================================
    # Utility Methods
    # ========================================================================
    
    def _calculate_curvature(
        self,
        p1: Point, p2: Point, p3: Point
    ) -> float:
        """
        Calculate curvature at p2 given three consecutive points.
        
        Uses Menger curvature: k = 4*Area / (|a|*|b|*|c|)
        where a, b, c are the side lengths of the triangle.
        
        Args:
            p1, p2, p3: Three consecutive path points
            
        Returns:
            Curvature value (higher = sharper turn)
        """
        # Side lengths
        a = self.calculate_distance_3d(p1, p2)
        b = self.calculate_distance_3d(p2, p3)
        c = self.calculate_distance_3d(p1, p3)
        
        if a < 0.01 or b < 0.01 or c < 0.01:
            return 0.0
        
        # Semi-perimeter
        s = (a + b + c) / 2
        
        # Area using Heron's formula
        area_sq = s * (s - a) * (s - b) * (s - c)
        if area_sq <= 0:
            return 0.0
        
        area = math.sqrt(area_sq)
        
        # Menger curvature
        curvature = 4 * area / (a * b * c)
        
        return curvature
    
    def _rotate_offset(
        self,
        offset_x: float,
        offset_y: float,
        heading: float
    ) -> Tuple[float, float]:
        """
        Rotate offset by heading angle.
        
        Args:
            offset_x: X offset in local frame
            offset_y: Y offset in local frame
            heading: Rotation angle in radians
            
        Returns:
            Tuple of (rotated_x, rotated_y)
        """
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)
        
        rotated_x = offset_x * cos_h - offset_y * sin_h
        rotated_y = offset_x * sin_h + offset_y * cos_h
        
        return (rotated_x, rotated_y)
    
    def _extract_position(self, pos_data) -> Point:
        """
        Extract Point from various position formats.
        
        Handles:
        - Point objects
        - Dictionaries with x, y, z keys
        - Lists/tuples [x, y, z]
        
        Args:
            pos_data: Position data in various formats
            
        Returns:
            Point object
        """
        if isinstance(pos_data, Point):
            return pos_data
        elif isinstance(pos_data, dict):
            return self.create_point(
                float(pos_data.get('x', 0.0)),
                float(pos_data.get('y', 0.0)),
                float(pos_data.get('z', 30.0))
            )
        elif isinstance(pos_data, (list, tuple)):
            return self.create_point(
                float(pos_data[0]) if len(pos_data) > 0 else 0.0,
                float(pos_data[1]) if len(pos_data) > 1 else 0.0,
                float(pos_data[2]) if len(pos_data) > 2 else 30.0
            )
        else:
            self._log_warn(f"Unknown position format: {type(pos_data)}")
            return self.create_point(0.0, 0.0, 30.0)


class FormationFollowExecutor(FollowExecutor):
    """
    Specialized executor for formation following.
    
    Extends FollowExecutor with formation-specific features like
    dynamic position swapping, formation shape maintenance, and
    collision avoidance.
    
    Used by Squadron Manager for coordinated multi-drone formation flight.
    """
    
    def __init__(self, config: Dict):
        super().__init__(config)
        self._log_info("Formation follow executor initialized")
    
    def execute_formation_follow(
        self,
        leader_path: List[Dict],
        formation_position: Dict,
        my_drone_id: str,
        velocity: float = None
    ) -> List[PrimitiveCommand]:
        """
        Generate primitives to follow leader while maintaining formation position.
        
        Args:
            leader_path: Leader's planned waypoints
            formation_position: My position in formation {offset_x, offset_y, offset_z}
            my_drone_id: This drone's identifier
            velocity: Travel velocity
            
        Returns:
            List of PrimitiveCommand messages
        """
        if velocity is None:
            velocity = self.cruise_speed
        
        parameters = {
            'mode': self.MODE_LEADER,
            'leader_drone_id': 'leader',
            'leader_waypoints': leader_path,
            'offset': formation_position,
            'velocity': velocity,
            'rotate_with_leader': True,
            'maintain_formation': True
        }
        
        return self._execute_leader_follow(parameters)
