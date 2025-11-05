#!/usr/bin/env python3
"""
Collision Avoidance - Basic spatial deconfliction for multi-drone operations
"""

from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import math


@dataclass
class Position3D:
    """3D position"""
    x: float
    y: float
    z: float
    
    def distance_to(self, other: 'Position3D') -> float:
        """Calculate Euclidean distance to another position"""
        return math.sqrt(
            (self.x - other.x)**2 +
            (self.y - other.y)**2 +
            (self.z - other.z)**2
        )


@dataclass
class ConflictInfo:
    """Information about a spatial conflict between drones"""
    drone_1: str
    drone_2: str
    time_index: int
    distance: float
    position_1: Position3D
    position_2: Position3D
    severity: str  # 'critical', 'warning', 'info'


class CollisionAvoidance:
    """
    Basic collision avoidance system for multi-drone operations.
    Performs spatial deconfliction by checking for potential conflicts.
    """
    
    def __init__(
        self,
        safety_radius: float = 5.0,
        warning_radius: float = 10.0,
        logger=None
    ):
        """
        Initialize collision avoidance.
        
        Args:
            safety_radius: Minimum safe separation (meters)
            warning_radius: Warning distance (meters)
            logger: Logger instance
        """
        self.safety_radius = safety_radius
        self.warning_radius = warning_radius
        self.logger = logger
    
    def check_current_positions(
        self,
        drone_positions: Dict[str, Tuple[float, float, float]]
    ) -> List[ConflictInfo]:
        """
        Check current drone positions for conflicts.
        
        Args:
            drone_positions: Dict mapping drone_id to (x, y, z) position
            
        Returns:
            List of detected conflicts
        """
        conflicts = []
        drone_ids = list(drone_positions.keys())
        
        # Check all pairs of drones
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                drone_1 = drone_ids[i]
                drone_2 = drone_ids[j]
                
                pos_1 = Position3D(*drone_positions[drone_1])
                pos_2 = Position3D(*drone_positions[drone_2])
                
                distance = pos_1.distance_to(pos_2)
                
                # Determine severity
                if distance < self.safety_radius:
                    severity = 'critical'
                elif distance < self.warning_radius:
                    severity = 'warning'
                else:
                    continue  # No conflict
                
                conflict = ConflictInfo(
                    drone_1=drone_1,
                    drone_2=drone_2,
                    time_index=0,  # Current time
                    distance=distance,
                    position_1=pos_1,
                    position_2=pos_2,
                    severity=severity
                )
                
                conflicts.append(conflict)
                
                if self.logger:
                    self.logger.warning(
                        f"[{severity.upper()}] Collision risk: {drone_1} and {drone_2} "
                        f"separated by {distance:.2f}m (min safe: {self.safety_radius:.2f}m)"
                    )
        
        return conflicts
    
    def check_path_conflicts(
        self,
        drone_paths: Dict[str, List[Tuple[float, float, float]]]
    ) -> List[ConflictInfo]:
        """
        Check planned paths for potential conflicts.
        
        Args:
            drone_paths: Dict mapping drone_id to list of (x, y, z) waypoints
            
        Returns:
            List of detected conflicts along paths
        """
        conflicts = []
        drone_ids = list(drone_paths.keys())
        
        # Find maximum path length
        max_length = max(len(path) for path in drone_paths.values())
        
        # Check all pairs at each time step
        for t in range(max_length):
            for i in range(len(drone_ids)):
                for j in range(i + 1, len(drone_ids)):
                    drone_1 = drone_ids[i]
                    drone_2 = drone_ids[j]
                    
                    path_1 = drone_paths[drone_1]
                    path_2 = drone_paths[drone_2]
                    
                    # Skip if either path doesn't have this time index
                    if t >= len(path_1) or t >= len(path_2):
                        continue
                    
                    pos_1 = Position3D(*path_1[t])
                    pos_2 = Position3D(*path_2[t])
                    
                    distance = pos_1.distance_to(pos_2)
                    
                    # Determine severity
                    if distance < self.safety_radius:
                        severity = 'critical'
                    elif distance < self.warning_radius:
                        severity = 'warning'
                    else:
                        continue
                    
                    conflict = ConflictInfo(
                        drone_1=drone_1,
                        drone_2=drone_2,
                        time_index=t,
                        distance=distance,
                        position_1=pos_1,
                        position_2=pos_2,
                        severity=severity
                    )
                    
                    conflicts.append(conflict)
                    
                    if self.logger:
                        self.logger.warning(
                            f"[{severity.upper()}] Path conflict at waypoint {t}: "
                            f"{drone_1} and {drone_2} separated by {distance:.2f}m"
                        )
        
        return conflicts
    
    def check_waypoint_separation(
        self,
        new_drone_id: str,
        new_waypoint: Tuple[float, float, float],
        active_drones: Dict[str, Tuple[float, float, float]]
    ) -> bool:
        """
        Check if a new waypoint maintains safe separation from active drones.
        
        Args:
            new_drone_id: ID of drone getting new waypoint
            new_waypoint: Target (x, y, z) position
            active_drones: Dict of other drone IDs to current positions
            
        Returns:
            True if waypoint is safe, False if conflicts exist
        """
        new_pos = Position3D(*new_waypoint)
        
        for drone_id, position in active_drones.items():
            if drone_id == new_drone_id:
                continue
            
            other_pos = Position3D(*position)
            distance = new_pos.distance_to(other_pos)
            
            if distance < self.safety_radius:
                if self.logger:
                    self.logger.error(
                        f"Waypoint rejected: Would put {new_drone_id} within "
                        f"{distance:.2f}m of {drone_id} (min: {self.safety_radius:.2f}m)"
                    )
                return False
        
        return True
    
    def suggest_safe_altitude_offset(
        self,
        target_waypoint: Tuple[float, float, float],
        nearby_drones: Dict[str, Tuple[float, float, float]],
        min_vertical_sep: float = 5.0
    ) -> Optional[float]:
        """
        Suggest altitude offset to avoid conflicts.
        
        Args:
            target_waypoint: Desired (x, y, z) waypoint
            nearby_drones: Dict of drone IDs to positions
            min_vertical_sep: Minimum vertical separation (meters)
            
        Returns:
            Suggested altitude offset, or None if no conflict
        """
        target_pos = Position3D(*target_waypoint)
        
        for drone_id, position in nearby_drones.items():
            other_pos = Position3D(*position)
            
            # Check horizontal distance
            horizontal_dist = math.sqrt(
                (target_pos.x - other_pos.x)**2 +
                (target_pos.y - other_pos.y)**2
            )
            
            # If horizontally close, check vertical separation
            if horizontal_dist < self.warning_radius:
                vertical_sep = abs(target_pos.z - other_pos.z)
                
                if vertical_sep < min_vertical_sep:
                    # Suggest offset to maintain vertical separation
                    if target_pos.z < other_pos.z:
                        suggested_offset = -(min_vertical_sep - vertical_sep + 1.0)
                    else:
                        suggested_offset = (min_vertical_sep - vertical_sep + 1.0)
                    
                    if self.logger:
                        self.logger.info(
                            f"Suggesting altitude offset of {suggested_offset:.1f}m "
                            f"to maintain separation from {drone_id}"
                        )
                    
                    return suggested_offset
        
        return None  # No conflict, no offset needed
    
    def get_separation_status(
        self,
        drone_positions: Dict[str, Tuple[float, float, float]]
    ) -> Dict:
        """
        Get comprehensive separation status for all drones.
        
        Returns:
            Dictionary with separation statistics
        """
        conflicts = self.check_current_positions(drone_positions)
        
        critical_conflicts = [c for c in conflicts if c.severity == 'critical']
        warning_conflicts = [c for c in conflicts if c.severity == 'warning']
        
        return {
            'num_drones': len(drone_positions),
            'total_conflicts': len(conflicts),
            'critical_conflicts': len(critical_conflicts),
            'warning_conflicts': len(warning_conflicts),
            'safety_radius': self.safety_radius,
            'warning_radius': self.warning_radius,
            'is_safe': len(critical_conflicts) == 0
        }
