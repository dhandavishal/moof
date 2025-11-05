#!/usr/bin/env python3
"""
Task Allocator - Allocates tasks to drones based on various strategies
"""

from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
from enum import Enum
import math

from squadron_manager.drone_registry import DroneInfo


class AllocationStrategy(Enum):
    """Task allocation strategies"""
    GREEDY = "greedy"  # First available drone
    NEAREST = "nearest"  # Closest drone to task location
    LOAD_BALANCED = "load_balanced"  # Balance workload across drones
    CAPABILITY_BASED = "capability_based"  # Match capabilities to requirements
    OPTIMAL = "optimal"  # Hungarian algorithm (future)


@dataclass
class TaskRequirements:
    """Requirements for a task"""
    min_battery: float = 20.0  # Minimum battery percentage
    min_flight_time: float = 5.0  # Minimum flight time in minutes
    requires_camera: bool = False
    requires_lidar: bool = False
    requires_gripper: bool = False
    max_speed_required: float = 3.0  # m/s
    target_location: Optional[Tuple[float, float, float]] = None  # (x, y, z)
    estimated_duration: float = 10.0  # minutes


class TaskAllocator:
    """
    Allocates tasks to drones using various strategies.
    Considers drone capabilities, battery, position, and current workload.
    """
    
    def __init__(self, logger, strategy: AllocationStrategy = AllocationStrategy.NEAREST):
        """Initialize task allocator"""
        self.logger = logger
        self.strategy = strategy
        self.task_assignments: Dict[str, str] = {}  # task_id -> drone_id
        
    def set_strategy(self, strategy: AllocationStrategy):
        """Change allocation strategy"""
        self.strategy = strategy
        self.logger.info(f"Task allocation strategy set to: {strategy.value}")
    
    def allocate_task(
        self,
        task_id: str,
        available_drones: List[DroneInfo],
        requirements: Optional[TaskRequirements] = None
    ) -> Optional[str]:
        """
        Allocate a task to the best available drone.
        
        Args:
            task_id: Unique task identifier
            available_drones: List of available drones
            requirements: Task requirements
            
        Returns:
            drone_id of selected drone, or None if no suitable drone found
        """
        if not available_drones:
            self.logger.warning(f"No available drones for task {task_id}")
            return None
        
        if requirements is None:
            requirements = TaskRequirements()
        
        # Filter drones that meet requirements
        suitable_drones = self._filter_suitable_drones(available_drones, requirements)
        
        if not suitable_drones:
            self.logger.warning(f"No suitable drones found for task {task_id}")
            return None
        
        # Select best drone based on strategy
        if self.strategy == AllocationStrategy.GREEDY:
            selected_drone = self._greedy_allocation(suitable_drones)
        elif self.strategy == AllocationStrategy.NEAREST:
            selected_drone = self._nearest_allocation(suitable_drones, requirements)
        elif self.strategy == AllocationStrategy.LOAD_BALANCED:
            selected_drone = self._load_balanced_allocation(suitable_drones)
        elif self.strategy == AllocationStrategy.CAPABILITY_BASED:
            selected_drone = self._capability_based_allocation(suitable_drones, requirements)
        else:
            self.logger.warning(f"Unknown strategy {self.strategy}, using greedy")
            selected_drone = self._greedy_allocation(suitable_drones)
        
        if selected_drone:
            self.task_assignments[task_id] = selected_drone.drone_id
            self.logger.info(
                f"Allocated task {task_id} to drone {selected_drone.drone_id} "
                f"using {self.strategy.value} strategy"
            )
            return selected_drone.drone_id
        
        return None
    
    def deallocate_task(self, task_id: str):
        """Remove task allocation"""
        if task_id in self.task_assignments:
            drone_id = self.task_assignments[task_id]
            del self.task_assignments[task_id]
            self.logger.info(f"Deallocated task {task_id} from drone {drone_id}")
    
    def get_allocation(self, task_id: str) -> Optional[str]:
        """Get drone assigned to a task"""
        return self.task_assignments.get(task_id)
    
    def _filter_suitable_drones(
        self,
        drones: List[DroneInfo],
        requirements: TaskRequirements
    ) -> List[DroneInfo]:
        """Filter drones that meet task requirements with energy awareness"""
        suitable = []
        
        for drone in drones:
            # Check basic battery threshold
            if drone.battery_percentage < requirements.min_battery:
                continue
            
            # Check flight time
            if drone.battery_remaining_time < requirements.min_flight_time:
                continue
            
            # Enhanced battery check: Estimate mission energy + RTL reserve
            estimated_mission_energy = self._estimate_mission_energy(drone, requirements)
            rtl_energy_reserve = self._estimate_rtl_energy(drone)
            safety_margin = 15.0  # 15% safety margin
            
            required_battery = estimated_mission_energy + rtl_energy_reserve + safety_margin
            
            if drone.battery_percentage < required_battery:
                self.logger.debug(
                    f"Drone {drone.drone_id} filtered: insufficient battery "
                    f"(has {drone.battery_percentage:.1f}%, needs {required_battery:.1f}%)"
                )
                continue
            
            # Check capabilities
            if requirements.requires_camera and not drone.capabilities.has_camera:
                continue
            
            if requirements.requires_lidar and not drone.capabilities.has_lidar:
                continue
            
            if requirements.requires_gripper and not drone.capabilities.has_gripper:
                continue
            
            # Check speed capability
            if drone.capabilities.max_speed < requirements.max_speed_required:
                continue
            
            suitable.append(drone)
        
        return suitable
    
    def _estimate_mission_energy(self, drone: DroneInfo, requirements: TaskRequirements) -> float:
        """
        Estimate energy required for mission as percentage of battery.
        
        Args:
            drone: Drone information
            requirements: Mission requirements
            
        Returns:
            Estimated battery percentage needed
        """
        # Simplified energy estimation
        # TODO: Implement more sophisticated model based on:
        # - Distance to waypoints
        # - Altitude changes
        # - Payload weight
        # - Weather conditions
        
        if requirements.target_location:
            # Calculate distance from current position to target
            dx = requirements.target_location[0] - drone.position_x
            dy = requirements.target_location[1] - drone.position_y
            dz = requirements.target_location[2] - drone.position_z
            distance = (dx**2 + dy**2 + dz**2) ** 0.5
            
            # Rough estimate: 1% battery per 50m horizontal + 1% per 20m vertical
            horizontal_dist = (dx**2 + dy**2) ** 0.5
            vertical_dist = abs(dz)
            
            energy_horizontal = (horizontal_dist / 50.0) * 1.0  # 1% per 50m
            energy_vertical = (vertical_dist / 20.0) * 2.0  # 2% per 20m (more expensive)
            
            return energy_horizontal + energy_vertical + 5.0  # +5% for maneuvers
        
        # Default estimate if no target location
        return 10.0  # 10% for unknown mission
    
    def _estimate_rtl_energy(self, drone: DroneInfo) -> float:
        """
        Estimate energy required for Return-To-Launch.
        
        Args:
            drone: Drone information
            
        Returns:
            Estimated battery percentage needed for RTL
        """
        # Estimate distance to home (assuming home is at origin)
        distance_to_home = (
            drone.position_x**2 +
            drone.position_y**2 +
            drone.position_z**2
        ) ** 0.5
        
        # Rough estimate: 1% per 50m
        rtl_energy = (distance_to_home / 50.0) * 1.0 + 3.0  # +3% for landing
        
        return max(rtl_energy, 8.0)  # Minimum 8% reserve for RTL
    
    def _greedy_allocation(self, drones: List[DroneInfo]) -> Optional[DroneInfo]:
        """
        Greedy strategy: Select first available drone.
        Simple and fast but may not be optimal.
        """
        return drones[0] if drones else None
    
    def _nearest_allocation(
        self,
        drones: List[DroneInfo],
        requirements: TaskRequirements
    ) -> Optional[DroneInfo]:
        """
        Nearest strategy: Select drone closest to task location.
        Minimizes travel time and energy consumption.
        """
        if not requirements.target_location:
            # No target location specified, fall back to greedy
            return self._greedy_allocation(drones)
        
        target_x, target_y, target_z = requirements.target_location
        
        min_distance = float('inf')
        nearest_drone = None
        
        for drone in drones:
            # Calculate 3D Euclidean distance
            distance = math.sqrt(
                (drone.position_x - target_x) ** 2 +
                (drone.position_y - target_y) ** 2 +
                (drone.position_z - target_z) ** 2
            )
            
            if distance < min_distance:
                min_distance = distance
                nearest_drone = drone
        
        if nearest_drone:
            self.logger.debug(
                f"Selected {nearest_drone.drone_id} (distance: {min_distance:.2f}m)"
            )
        
        return nearest_drone
    
    def _load_balanced_allocation(self, drones: List[DroneInfo]) -> Optional[DroneInfo]:
        """
        Load balanced strategy: Select drone with highest remaining capacity.
        Considers battery percentage and current workload.
        """
        best_score = -1
        best_drone = None
        
        for drone in drones:
            # Score based on battery and workload
            # Higher battery = higher score
            # No current task = bonus points
            score = drone.battery_percentage
            
            if drone.current_task_id is None:
                score += 20.0  # Bonus for idle drone
            
            if score > best_score:
                best_score = score
                best_drone = drone
        
        if best_drone:
            self.logger.debug(
                f"Selected {best_drone.drone_id} (score: {best_score:.2f}, "
                f"battery: {best_drone.battery_percentage:.1f}%)"
            )
        
        return best_drone
    
    def _capability_based_allocation(
        self,
        drones: List[DroneInfo],
        requirements: TaskRequirements
    ) -> Optional[DroneInfo]:
        """
        Capability-based strategy: Select drone with best matching capabilities.
        Prioritizes specialized capabilities over general ones.
        """
        best_score = -1
        best_drone = None
        
        for drone in drones:
            score = 0.0
            
            # Score based on capability matching
            if requirements.requires_camera and drone.capabilities.has_camera:
                score += 10.0
            
            if requirements.requires_lidar and drone.capabilities.has_lidar:
                score += 10.0
            
            if requirements.requires_gripper and drone.capabilities.has_gripper:
                score += 10.0
            
            # Speed capability score (normalized)
            speed_ratio = requirements.max_speed_required / drone.capabilities.max_speed
            score += (1.0 - speed_ratio) * 5.0  # Prefer faster drones
            
            # Battery score (normalized)
            score += (drone.battery_percentage / 100.0) * 10.0
            
            # Range score
            estimated_range = drone.get_estimated_range()
            if requirements.target_location:
                target_x, target_y, target_z = requirements.target_location
                distance = math.sqrt(
                    (drone.position_x - target_x) ** 2 +
                    (drone.position_y - target_y) ** 2 +
                    (drone.position_z - target_z) ** 2
                )
                # Can the drone reach target and return?
                range_ratio = (distance * 2.0) / estimated_range
                if range_ratio < 1.0:
                    score += (1.0 - range_ratio) * 10.0
            
            if score > best_score:
                best_score = score
                best_drone = drone
        
        if best_drone:
            self.logger.debug(
                f"Selected {best_drone.drone_id} (capability score: {best_score:.2f})"
            )
        
        return best_drone
    
    def get_allocation_summary(self) -> Dict:
        """Get summary of current task allocations"""
        return {
            'total_allocations': len(self.task_assignments),
            'strategy': self.strategy.value,
            'allocations': dict(self.task_assignments)
        }
