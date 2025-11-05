#!/usr/bin/env python3
"""
Formation Controller - Manages multi-drone formations
"""

from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import math

from squadron_manager.drone_registry import DroneInfo


class FormationType(Enum):
    """Formation types"""
    LINE = "line"  # Drones in a line
    WEDGE = "wedge"  # V-formation
    GRID = "grid"  # Square grid
    CIRCLE = "circle"  # Circular formation
    COLUMN = "column"  # Vertical column
    CUSTOM = "custom"  # Custom positions


@dataclass
class FormationParameters:
    """Parameters for formation"""
    formation_type: FormationType = FormationType.LINE
    spacing: float = 10.0  # meters between drones
    altitude: float = 50.0  # formation altitude (meters)
    heading: float = 0.0  # formation heading (radians)
    leader_id: Optional[str] = None  # Leader drone for leader-follower
    center: Tuple[float, float, float] = (0.0, 0.0, 50.0)  # Formation center


class FormationController:
    """
    Controls and maintains multi-drone formations.
    Supports various formation types and can adapt to drone availability.
    """
    
    def __init__(self, logger):
        """Initialize formation controller"""
        self.logger = logger
        self.active_formation: Optional[FormationParameters] = None
        self.formation_positions: Dict[str, Tuple[float, float, float]] = {}
        
    def create_formation(
        self,
        drones: List[DroneInfo],
        params: FormationParameters
    ) -> bool:
        """
        Create a formation with the given drones.
        
        Args:
            drones: List of drones to include in formation
            params: Formation parameters
            
        Returns:
            True if formation created successfully
        """
        if len(drones) < 2:
            self.logger.error("Need at least 2 drones for formation")
            return False
        
        self.active_formation = params
        
        # Calculate positions based on formation type
        if params.formation_type == FormationType.LINE:
            positions = self._calculate_line_formation(drones, params)
        elif params.formation_type == FormationType.WEDGE:
            positions = self._calculate_wedge_formation(drones, params)
        elif params.formation_type == FormationType.GRID:
            positions = self._calculate_grid_formation(drones, params)
        elif params.formation_type == FormationType.CIRCLE:
            positions = self._calculate_circle_formation(drones, params)
        elif params.formation_type == FormationType.COLUMN:
            positions = self._calculate_column_formation(drones, params)
        else:
            self.logger.error(f"Unknown formation type: {params.formation_type}")
            return False
        
        self.formation_positions = positions
        
        self.logger.info(
            f"Created {params.formation_type.value} formation with {len(drones)} drones "
            f"(spacing: {params.spacing}m, altitude: {params.altitude}m)"
        )
        
        return True
    
    def get_target_position(self, drone_id: str) -> Optional[Tuple[float, float, float]]:
        """Get target position for a drone in formation"""
        return self.formation_positions.get(drone_id)
    
    def get_all_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Get all target positions in formation"""
        return dict(self.formation_positions)
    
    def update_formation_center(self, center: Tuple[float, float, float]):
        """Update formation center (e.g., following a moving target)"""
        if not self.active_formation:
            return
        
        old_center = self.active_formation.center
        self.active_formation.center = center
        
        # Translate all positions
        dx = center[0] - old_center[0]
        dy = center[1] - old_center[1]
        dz = center[2] - old_center[2]
        
        for drone_id in self.formation_positions:
            x, y, z = self.formation_positions[drone_id]
            self.formation_positions[drone_id] = (x + dx, y + dy, z + dz)
    
    def remove_drone_from_formation(self, drone_id: str) -> bool:
        """Remove a drone from the formation and rebalance"""
        if drone_id not in self.formation_positions:
            return False
        
        del self.formation_positions[drone_id]
        self.logger.info(f"Removed {drone_id} from formation")
        return True
    
    def add_drone_to_formation(self, drone: DroneInfo) -> bool:
        """Add a new drone to the formation"""
        if not self.active_formation:
            self.logger.error("No active formation")
            return False
        
        # Find best position for new drone (at the end of formation)
        # This is simplified - more sophisticated logic could be added
        num_drones = len(self.formation_positions) + 1
        
        if self.active_formation.formation_type == FormationType.LINE:
            # Add to the end of the line
            offset = (num_drones - 1) * self.active_formation.spacing
            x = self.active_formation.center[0] + offset
            y = self.active_formation.center[1]
            z = self.active_formation.altitude
            self.formation_positions[drone.drone_id] = (x, y, z)
        else:
            # For other formations, recalculate entire formation
            # (simplified approach)
            drones = [drone]  # Add new drone
            self.create_formation(drones, self.active_formation)
        
        self.logger.info(f"Added {drone.drone_id} to formation")
        return True
    
    def check_formation_integrity(self, drones: List[DroneInfo]) -> Dict[str, float]:
        """
        Check how well drones are maintaining formation.
        
        Returns:
            Dict mapping drone_id to distance error from target position
        """
        errors = {}
        
        for drone in drones:
            if drone.drone_id not in self.formation_positions:
                continue
            
            target_x, target_y, target_z = self.formation_positions[drone.drone_id]
            
            error = math.sqrt(
                (drone.position_x - target_x) ** 2 +
                (drone.position_y - target_y) ** 2 +
                (drone.position_z - target_z) ** 2
            )
            
            errors[drone.drone_id] = error
        
        return errors
    
    def _calculate_line_formation(
        self,
        drones: List[DroneInfo],
        params: FormationParameters
    ) -> Dict[str, Tuple[float, float, float]]:
        """Calculate positions for line formation"""
        positions = {}
        center_x, center_y, _ = params.center
        
        # Arrange drones in a line along X-axis
        for i, drone in enumerate(drones):
            offset = i * params.spacing
            x = center_x + offset
            y = center_y
            z = params.altitude
            positions[drone.drone_id] = (x, y, z)
        
        return positions
    
    def _calculate_wedge_formation(
        self,
        drones: List[DroneInfo],
        params: FormationParameters
    ) -> Dict[str, Tuple[float, float, float]]:
        """Calculate positions for wedge (V) formation"""
        positions = {}
        center_x, center_y, _ = params.center
        
        # Leader at the front
        if params.leader_id and params.leader_id in [d.drone_id for d in drones]:
            leader_idx = next(i for i, d in enumerate(drones) if d.drone_id == params.leader_id)
            drones[0], drones[leader_idx] = drones[leader_idx], drones[0]
        
        # First drone is leader
        positions[drones[0].drone_id] = (center_x, center_y, params.altitude)
        
        # Arrange others in V shape
        for i in range(1, len(drones)):
            side = 1 if i % 2 == 1 else -1  # Alternate left/right
            row = (i + 1) // 2
            
            x = center_x - row * params.spacing * 0.7  # Offset backward
            y = center_y + side * row * params.spacing * 0.5  # Offset sideways
            z = params.altitude
            
            positions[drones[i].drone_id] = (x, y, z)
        
        return positions
    
    def _calculate_grid_formation(
        self,
        drones: List[DroneInfo],
        params: FormationParameters
    ) -> Dict[str, Tuple[float, float, float]]:
        """Calculate positions for grid formation"""
        positions = {}
        center_x, center_y, _ = params.center
        
        # Calculate grid dimensions (try to make it square)
        n = len(drones)
        cols = int(math.ceil(math.sqrt(n)))
        rows = int(math.ceil(n / cols))
        
        # Center the grid
        start_x = center_x - (cols - 1) * params.spacing / 2
        start_y = center_y - (rows - 1) * params.spacing / 2
        
        for i, drone in enumerate(drones):
            row = i // cols
            col = i % cols
            
            x = start_x + col * params.spacing
            y = start_y + row * params.spacing
            z = params.altitude
            
            positions[drone.drone_id] = (x, y, z)
        
        return positions
    
    def _calculate_circle_formation(
        self,
        drones: List[DroneInfo],
        params: FormationParameters
    ) -> Dict[str, Tuple[float, float, float]]:
        """Calculate positions for circular formation"""
        positions = {}
        center_x, center_y, _ = params.center
        
        # Radius based on spacing
        n = len(drones)
        radius = params.spacing * n / (2 * math.pi)
        
        for i, drone in enumerate(drones):
            angle = (2 * math.pi * i / n) + params.heading
            
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            z = params.altitude
            
            positions[drone.drone_id] = (x, y, z)
        
        return positions
    
    def _calculate_column_formation(
        self,
        drones: List[DroneInfo],
        params: FormationParameters
    ) -> Dict[str, Tuple[float, float, float]]:
        """Calculate positions for vertical column formation"""
        positions = {}
        center_x, center_y, center_z = params.center
        
        # Arrange drones vertically
        for i, drone in enumerate(drones):
            x = center_x
            y = center_y
            z = center_z + i * params.spacing
            
            positions[drone.drone_id] = (x, y, z)
        
        return positions
    
    def get_formation_waypoints(
        self,
        path: List[Tuple[float, float, float]]
    ) -> Dict[str, List[Tuple[float, float, float]]]:
        """
        Generate waypoints for each drone to maintain formation along a path.
        
        Args:
            path: List of waypoints for formation center
            
        Returns:
            Dict mapping drone_id to list of waypoints
        """
        if not self.active_formation or not self.formation_positions:
            return {}
        
        drone_waypoints = {drone_id: [] for drone_id in self.formation_positions}
        
        # For each path waypoint, calculate offset for each drone
        for waypoint in path:
            # Update formation center
            old_center = self.active_formation.center
            offset_x = waypoint[0] - old_center[0]
            offset_y = waypoint[1] - old_center[1]
            offset_z = waypoint[2] - old_center[2]
            
            # Calculate new position for each drone
            for drone_id, (x, y, z) in self.formation_positions.items():
                new_x = x + offset_x
                new_y = y + offset_y
                new_z = z + offset_z
                drone_waypoints[drone_id].append((new_x, new_y, new_z))
        
        return drone_waypoints
