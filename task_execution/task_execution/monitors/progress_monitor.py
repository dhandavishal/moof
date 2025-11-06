#!/usr/bin/env python3
"""
Mission progress monitor for ROS2.

Monitors mission execution progress, tracks completion, estimates time
remaining, and publishes updates.
"""

from typing import Optional, List, Dict, Any
from dataclasses import dataclass
import math

from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32

try:
    from shapely.geometry import Polygon, Point as ShapelyPoint
    from shapely.ops import unary_union
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False


@dataclass
class ProgressReport:
    """Mission progress report"""
    task_id: str
    task_type: str
    completion_percentage: float
    waypoints_completed: int
    waypoints_total: int
    area_covered_m2: float
    area_total_m2: float
    primitives_completed: int
    primitives_total: int
    eta_seconds: Optional[float]
    elapsed_seconds: float


class ProgressMonitor:
    """
    Monitors mission execution progress.
    Tracks completion, estimates time remaining, and publishes updates.
    """
    
    def __init__(self, node: Node, namespace: str = ''):
        """
        Initialize progress monitor.
        
        Args:
            node: ROS2 node instance
            namespace: Drone namespace
        """
        self.node = node
        self.namespace = namespace
        
        # Current mission tracking
        self.active_task_id: Optional[str] = None
        self.active_task_type: Optional[str] = None
        self.active_task_params: Optional[Dict[str, Any]] = None
        self.mission_start_time: Optional[float] = None
        
        # Progress metrics
        self.waypoints_completed = 0
        self.waypoints_total = 0
        self.primitives_completed = 0
        self.primitives_total = 0
        
        # Survey-specific
        self.survey_area_total = 0.0
        self.survey_area_covered = 0.0
        self.covered_polygons: List[Polygon] = []
        
        # Publisher
        progress_topic = f'/{namespace}/tee/progress' if namespace else '/tee/progress'
        self.progress_pub = self.node.create_publisher(
            Float32,
            progress_topic,
            10
        )
        
        self.node.get_logger().info("Progress monitor initialized")
    
    def start_mission(self, 
                     task_id: str,
                     task_type: str,
                     task_parameters: Dict[str, Any],
                     total_primitives: int):
        """
        Start tracking a new mission.
        
        Args:
            task_id: Unique task identifier
            task_type: Type of task (waypoint, survey, search, etc.)
            task_parameters: Task parameters dictionary
            total_primitives: Total number of primitives in task
        """
        self.active_task_id = task_id
        self.active_task_type = task_type
        self.active_task_params = task_parameters
        self.mission_start_time = self.node.get_clock().now().nanoseconds / 1e9
        
        # Reset counters
        self.waypoints_completed = 0
        self.primitives_completed = 0
        self.primitives_total = total_primitives
        
        # Task-specific initialization
        if task_type == 'waypoint':
            waypoints = task_parameters.get('waypoints', [])
            self.waypoints_total = len(waypoints)
        
        elif task_type == 'survey':
            area = task_parameters.get('area', [])
            if len(area) >= 3 and SHAPELY_AVAILABLE:
                self.survey_area_total = self._calculate_polygon_area(area)
                self.survey_area_covered = 0.0
                self.covered_polygons = []
            else:
                self.survey_area_total = 0.0
        
        self.node.get_logger().info(
            f"Started tracking mission {task_id}: {total_primitives} primitives"
        )
    
    def update_primitive_completed(self, primitive_index: int):
        """
        Update when a primitive completes.
        
        Args:
            primitive_index: Index of completed primitive
        """
        self.primitives_completed = primitive_index + 1
        
        # Publish progress
        self._publish_progress()
    
    def update_waypoint_reached(self, waypoint_index: int):
        """
        Update when a waypoint is reached.
        
        Args:
            waypoint_index: Index of reached waypoint
        """
        self.waypoints_completed = waypoint_index + 1
        self.node.get_logger().debug(
            f"Waypoint {self.waypoints_completed}/{self.waypoints_total} reached"
        )
    
    def update_survey_position(self, current_position: Point, camera_fov_m: float):
        """
        Update survey coverage based on current position.
        
        Args:
            current_position: Current drone position
            camera_fov_m: Camera field of view in meters
        """
        if not self.active_task_type or self.active_task_type != 'survey':
            return
        
        if not SHAPELY_AVAILABLE:
            self.node.get_logger().warn("Shapely not available - cannot track survey coverage")
            return
        
        # Create footprint polygon for current position
        footprint = self._create_camera_footprint(current_position, camera_fov_m)
        self.covered_polygons.append(footprint)
        
        # Calculate total covered area (union of all footprints)
        if len(self.covered_polygons) > 1:
            try:
                covered_union = unary_union(self.covered_polygons)
                self.survey_area_covered = covered_union.area
            except Exception as e:
                self.node.get_logger().warn(f"Failed to calculate survey coverage: {e}")
        else:
            self.survey_area_covered = footprint.area
        
        # Publish progress
        self._publish_progress()
    
    def get_completion_percentage(self) -> float:
        """
        Calculate overall completion percentage.
        
        Returns:
            Completion percentage (0.0 to 100.0)
        """
        if not self.active_task_type:
            return 0.0
        
        if self.active_task_type == 'survey':
            # Use area coverage
            if self.survey_area_total > 0:
                return (self.survey_area_covered / self.survey_area_total) * 100.0
            else:
                return 0.0
        
        elif self.active_task_type == 'waypoint':
            # Use waypoint completion
            if self.waypoints_total > 0:
                return (self.waypoints_completed / self.waypoints_total) * 100.0
            else:
                return 0.0
        
        else:
            # Use primitive completion
            if self.primitives_total > 0:
                return (self.primitives_completed / self.primitives_total) * 100.0
            else:
                return 0.0
    
    def estimate_time_remaining(self) -> Optional[float]:
        """
        Estimate time remaining in seconds.
        
        Returns:
            Estimated seconds remaining, or None if cannot estimate
        """
        if not self.mission_start_time or not self.active_task_id:
            return None
        
        # Calculate elapsed time
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.mission_start_time
        
        if elapsed < 1.0:
            return None  # Too early to estimate
        
        # Get completion percentage
        completion = self.get_completion_percentage() / 100.0
        
        if completion <= 0.0:
            return None
        
        # Linear extrapolation
        total_estimated = elapsed / completion
        remaining = total_estimated - elapsed
        
        return max(0.0, remaining)
    
    def get_progress_report(self) -> Optional[ProgressReport]:
        """
        Generate comprehensive progress report.
        
        Returns:
            ProgressReport object or None if no active mission
        """
        if not self.active_task_id or not self.mission_start_time:
            return None
        
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.mission_start_time
        eta = self.estimate_time_remaining()
        
        report = ProgressReport(
            task_id=self.active_task_id,
            task_type=self.active_task_type,
            completion_percentage=self.get_completion_percentage(),
            waypoints_completed=self.waypoints_completed,
            waypoints_total=self.waypoints_total,
            area_covered_m2=self.survey_area_covered,
            area_total_m2=self.survey_area_total,
            primitives_completed=self.primitives_completed,
            primitives_total=self.primitives_total,
            eta_seconds=eta,
            elapsed_seconds=elapsed
        )
        
        return report
    
    def _publish_progress(self):
        """Publish progress update"""
        completion = self.get_completion_percentage()
        msg = Float32()
        msg.data = completion
        self.progress_pub.publish(msg)
        
        # Log progress at milestones
        if completion % 25.0 < 1.0:  # Every 25%
            self.node.get_logger().info(f"Mission progress: {completion:.1f}%")
    
    def _calculate_polygon_area(self, polygon_coords: List) -> float:
        """
        Calculate polygon area using Shapely.
        
        Args:
            polygon_coords: List of [x, y] or [x, y, z] coordinates
            
        Returns:
            Area in square meters
        """
        if not SHAPELY_AVAILABLE or len(polygon_coords) < 3:
            return 0.0
        
        # Extract x, y coordinates - handle both dict and list/tuple formats
        coords_2d = []
        for p in polygon_coords:
            if isinstance(p, dict):
                coords_2d.append((p.get('x', 0.0), p.get('y', 0.0)))
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                coords_2d.append((p[0], p[1]))
            else:
                # Skip invalid points
                continue
        
        if len(coords_2d) < 3:
            return 0.0
            
        polygon = Polygon(coords_2d)
        return polygon.area
    
    def _create_camera_footprint(self, position: Point, fov_m: float) -> Polygon:
        """
        Create square camera footprint polygon.
        
        Args:
            position: Camera position
            fov_m: Field of view in meters (side length)
            
        Returns:
            Footprint polygon
        """
        half_fov = fov_m / 2.0
        
        coords = [
            (position.x - half_fov, position.y - half_fov),
            (position.x + half_fov, position.y - half_fov),
            (position.x + half_fov, position.y + half_fov),
            (position.x - half_fov, position.y + half_fov)
        ]
        
        return Polygon(coords)
    
    def complete_mission(self):
        """Mark current mission as complete"""
        if self.active_task_id and self.mission_start_time:
            current_time = self.node.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.mission_start_time
            self.node.get_logger().info(
                f"Mission {self.active_task_id} completed in {elapsed:.1f}s"
            )
        
        # Reset
        self.active_task_id = None
        self.active_task_type = None
        self.active_task_params = None
        self.mission_start_time = None
