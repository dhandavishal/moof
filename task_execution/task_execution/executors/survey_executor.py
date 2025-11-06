#!/usr/bin/env python3
"""
Survey executor with photogrammetry support.

Generates lawn-mower survey patterns optimized for photogrammetry.
"""

from typing import Dict, Any, List, Tuple
from geometry_msgs.msg import Point
import math

try:
    from shapely.geometry import Polygon, LineString
    from shapely.ops import unary_union
    SHAPELY_AVAILABLE = True
except ImportError:
    SHAPELY_AVAILABLE = False

from task_execution.executors.base_executor import BaseExecutor
from multi_drone_msgs.msg import PrimitiveCommand


class SurveyExecutor(BaseExecutor):
    """
    Executor for aerial survey tasks with photogrammetry calculations.
    
    Generates optimized lawn-mower patterns for complete area coverage
    with configurable overlap for photogrammetry.
    """
    
    def __init__(self, config: Dict):
        """
        Initialize survey executor.
        
        Args:
            config: Configuration dictionary
        """
        super().__init__(config)
        
        if not SHAPELY_AVAILABLE:
            self._log_warn("Shapely not available - advanced geometry features disabled")
        
        # Get survey-specific configuration
        survey_config = self.task_config.get('survey', {})
        self.default_overlap = survey_config.get('overlap', 0.7)
        self.default_sidelap = survey_config.get('sidelap', 0.6)
        self.default_altitude = survey_config.get('altitude', 50.0)
        
        self.cruise_speed = self.drone_config.get('cruise_speed', 5.0)
        
        self._log_info("Survey executor initialized")
    
    def execute(self, parameters: Dict[str, Any]) -> List[PrimitiveCommand]:
        """
        Generate primitive command sequence for survey mission.
        
        Expected parameters:
            - area: Survey area definition (polygon vertices or bounds)
            - altitude: Survey altitude in meters
            - orientation: Flight line orientation in degrees (0-180)
            - overlap: Forward overlap (0.0-1.0)
            - sidelap: Side overlap (0.0-1.0)
            - velocity: Optional cruise velocity
            - camera: Optional camera parameters for GSD calculation
                - sensor_width: Sensor width in mm
                - sensor_height: Sensor height in mm
                - focal_length: Focal length in mm
                - image_width: Image width in pixels
                - image_height: Image height in pixels
            - gsd: Optional ground sampling distance in cm/pixel
            
        Args:
            parameters: Task parameters
            
        Returns:
            List of PrimitiveCommand messages
        """
        area = parameters.get('area', [])
        altitude = parameters.get('altitude', self.default_altitude)
        orientation = parameters.get('orientation', None)
        overlap = parameters.get('overlap', self.default_overlap)
        sidelap = parameters.get('sidelap', self.default_sidelap)
        velocity = parameters.get('velocity', self.cruise_speed)
        camera = parameters.get('camera', None)
        desired_gsd = parameters.get('gsd', None)
        
        if not area:
            self._log_error("Survey executor received empty area definition")
            return []
        
        # Calculate or validate altitude for desired GSD
        if desired_gsd and camera:
            calculated_altitude = self._calculate_altitude_for_gsd(
                desired_gsd, camera
            )
            if abs(calculated_altitude - altitude) > 5.0:
                self._log_warn(
                    f"Requested altitude {altitude}m differs from calculated "
                    f"{calculated_altitude:.1f}m for GSD {desired_gsd}cm/px"
                )
                altitude = calculated_altitude
        
        # Calculate footprint and spacing
        if camera:
            footprint = self._calculate_footprint(altitude, camera)
            self._log_info(
                f"Camera footprint at {altitude}m: "
                f"{footprint[0]:.1f}m x {footprint[1]:.1f}m"
            )
        else:
            # Use default footprint estimation
            footprint = (altitude * 1.2, altitude * 0.9)  # Rough 4:3 aspect
            self._log_warn("No camera parameters - using estimated footprint")
        
        # Calculate flight line spacing
        line_spacing = footprint[1] * (1.0 - sidelap)
        photo_spacing = footprint[0] * (1.0 - overlap)
        
        self._log_info(
            f"Survey parameters: {overlap*100:.0f}% overlap, {sidelap*100:.0f}% sidelap, "
            f"line spacing {line_spacing:.1f}m, photo spacing {photo_spacing:.1f}m"
        )
        
        # Determine optimal orientation if not specified
        if orientation is None:
            orientation = self._calculate_optimal_orientation(area)
            self._log_info(f"Calculated optimal orientation: {orientation:.1f}°")
        
        # Generate flight lines
        flight_lines = self._generate_flight_lines(
            area, altitude, orientation, line_spacing
        )
        
        if not flight_lines:
            self._log_error("Failed to generate flight lines")
            return []
        
        # Optimize flight line order
        ordered_lines = self._optimize_flight_line_order(flight_lines)
        
        # Convert to waypoints
        waypoints = []
        for line in ordered_lines:
            for point in line:
                waypoints.append(point)
        
        self._log_info(
            f"Generated {len(waypoints)} waypoints across {len(ordered_lines)} flight lines"
        )
        
        # Generate primitive commands
        primitives = []
        for i, wp in enumerate(waypoints):
            goto_primitive = self.create_goto_primitive(
                position=wp,
                velocity=velocity,
                acceptance_radius=2.0
            )
            primitives.append(goto_primitive)
        
        self._log_info(f"Generated {len(primitives)} primitive commands for survey")
        return primitives
    
    def _calculate_altitude_for_gsd(self, 
                                    desired_gsd_cm: float,
                                    camera: Dict) -> float:
        """
        Calculate required altitude for desired ground sampling distance.
        
        GSD = (sensor_width * altitude * 100) / (focal_length * image_width)
        
        Args:
            desired_gsd_cm: Desired GSD in cm/pixel
            camera: Camera parameters
            
        Returns:
            Required altitude in meters
        """
        sensor_width = camera.get('sensor_width', 13.2)  # mm
        focal_length = camera.get('focal_length', 16.0)  # mm
        image_width = camera.get('image_width', 4000)    # pixels
        
        # altitude = (GSD * focal_length * image_width) / (sensor_width * 100)
        altitude = (desired_gsd_cm * focal_length * image_width) / (sensor_width * 100)
        
        self._log_debug(
            f"Calculated altitude {altitude:.1f}m for GSD {desired_gsd_cm}cm/px"
        )
        return altitude
    
    def _calculate_footprint(self, altitude: float, camera: Dict) -> Tuple[float, float]:
        """
        Calculate ground footprint dimensions.
        
        Args:
            altitude: Altitude in meters
            camera: Camera parameters
            
        Returns:
            (width, height) footprint in meters
        """
        sensor_width = camera.get('sensor_width', 13.2)   # mm
        sensor_height = camera.get('sensor_height', 8.8)  # mm
        focal_length = camera.get('focal_length', 16.0)   # mm
        
        # footprint = (sensor_dimension * altitude) / focal_length
        width = (sensor_width * altitude) / focal_length
        height = (sensor_height * altitude) / focal_length
        
        return (width, height)
    
    def _calculate_optimal_orientation(self, area: List) -> float:
        """
        Calculate optimal flight line orientation to minimize turns.
        
        Args:
            area: Area polygon vertices
            
        Returns:
            Optimal orientation in degrees
        """
        if not SHAPELY_AVAILABLE or len(area) < 3:
            return 0.0
        
        # Create polygon
        polygon = Polygon([(p[0], p[1]) for p in area])
        
        # Get minimum rotated rectangle (bounding box)
        min_rect = polygon.minimum_rotated_rectangle
        
        # Get rectangle coordinates
        coords = list(min_rect.exterior.coords)
        
        # Calculate edge angles
        angles = []
        for i in range(len(coords) - 1):
            dx = coords[i+1][0] - coords[i][0]
            dy = coords[i+1][1] - coords[i][1]
            angle = math.degrees(math.atan2(dy, dx))
            # Normalize to 0-180
            if angle < 0:
                angle += 180
            angles.append(angle)
        
        # Choose the orientation of the longest edge
        edge_lengths = [
            math.sqrt((coords[i+1][0] - coords[i][0])**2 + 
                     (coords[i+1][1] - coords[i][1])**2)
            for i in range(len(coords) - 1)
        ]
        
        longest_idx = edge_lengths.index(max(edge_lengths))
        optimal_angle = angles[longest_idx]
        
        # Flight lines perpendicular to long edge
        optimal_orientation = (optimal_angle + 90) % 180
        
        return optimal_orientation
    
    def _generate_flight_lines(self,
                               area: List,
                               altitude: float,
                               orientation: float,
                               spacing: float) -> List[List[Point]]:
        """
        Generate lawn-mower flight lines covering the area.
        
        Args:
            area: Area polygon vertices [(x,y,z)] or [(x,y)]
            altitude: Flight altitude
            orientation: Flight line orientation in degrees
            spacing: Line spacing in meters
            
        Returns:
            List of flight lines, each being a list of Points
        """
        if not area:
            return []
        
        # Convert area to 2D points - handle both dict and list/tuple formats
        area_2d = []
        for p in area:
            if isinstance(p, dict):
                area_2d.append((p.get('x', 0.0), p.get('y', 0.0)))
            elif isinstance(p, (list, tuple)) and len(p) >= 2:
                area_2d.append((p[0], p[1]))
            else:
                self._log_error(f"Invalid area point format: {p}")
                return []
        
        # Calculate bounds
        min_x = min(p[0] for p in area_2d)
        max_x = max(p[0] for p in area_2d)
        min_y = min(p[1] for p in area_2d)
        max_y = max(p[1] for p in area_2d)
        
        # Create expanded bounding box
        margin = spacing * 2
        bbox = [
            (min_x - margin, min_y - margin),
            (max_x + margin, min_y - margin),
            (max_x + margin, max_y + margin),
            (min_x - margin, max_y + margin)
        ]
        
        # Generate parallel lines across bounding box
        angle_rad = math.radians(orientation)
        
        # Rotate bounding box
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        
        # Calculate number of lines needed
        bbox_width = math.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)
        num_lines = int(bbox_width / spacing) + 2
        
        flight_lines = []
        
        for i in range(num_lines):
            offset = (i - num_lines/2) * spacing
            
            # Create line perpendicular to orientation
            line_length = bbox_width * 2
            
            # Line center point
            cx = center_x + offset * math.cos(angle_rad)
            cy = center_y + offset * math.sin(angle_rad)
            
            # Line endpoints
            perp_angle = angle_rad + math.pi/2
            dx = line_length/2 * math.cos(perp_angle)
            dy = line_length/2 * math.sin(perp_angle)
            
            p1 = Point(x=cx - dx, y=cy - dy, z=altitude)
            p2 = Point(x=cx + dx, y=cy + dy, z=altitude)
            
            flight_lines.append([p1, p2])
        
        return flight_lines
    
    def _optimize_flight_line_order(self, 
                                    flight_lines: List[List[Point]]) -> List[List[Point]]:
        """
        Optimize flight line order to minimize transit distance.
        
        Uses lawn-mower pattern (alternating directions).
        
        Args:
            flight_lines: Unordered flight lines
            
        Returns:
            Ordered flight lines
        """
        if not flight_lines:
            return []
        
        ordered = []
        reverse = False
        
        for line in flight_lines:
            if reverse:
                ordered.append(list(reversed(line)))
            else:
                ordered.append(line)
            reverse = not reverse
        
        return ordered
