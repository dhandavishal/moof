#!/usr/bin/env python3
"""
Task Allocator - Allocates tasks to drones based on various strategies
"""

from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass
from enum import Enum
import math

try:
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

from squadron_manager.drone_registry import DroneInfo


class AllocationStrategy(Enum):
    """Task allocation strategies"""
    GREEDY = "greedy"  # First available drone
    NEAREST = "nearest"  # Closest drone to task location
    LOAD_BALANCED = "load_balanced"  # Balance workload across drones
    CAPABILITY_BASED = "capability_based"  # Match capabilities to requirements
    OPTIMAL = "optimal"  # Hungarian algorithm for globally optimal assignment


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
        self.drone_task_count: Dict[str, int] = {}  # drone_id -> task count
        
        # Check if optimal allocation is available
        if strategy == AllocationStrategy.OPTIMAL and not SCIPY_AVAILABLE:
            self.logger.warning(
                "scipy not available, falling back to nearest allocation strategy"
            )
            self.strategy = AllocationStrategy.NEAREST
        
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
        elif self.strategy == AllocationStrategy.OPTIMAL:
            # For single task, optimal falls back to nearest
            selected_drone = self._nearest_allocation(suitable_drones, requirements)
        else:
            self.logger.warning(f"Unknown strategy {self.strategy}, using greedy")
            selected_drone = self._greedy_allocation(suitable_drones)
        
        if selected_drone:
            self.task_assignments[task_id] = selected_drone.drone_id
            # Track task count per drone
            self.drone_task_count[selected_drone.drone_id] = \
                self.drone_task_count.get(selected_drone.drone_id, 0) + 1
            self.logger.info(
                f"Allocated task {task_id} to drone {selected_drone.drone_id} "
                f"using {self.strategy.value} strategy"
            )
            return selected_drone.drone_id
        
        return None
    
    def allocate_multiple_tasks(
        self,
        tasks: List[Tuple[str, TaskRequirements]],
        available_drones: List[DroneInfo]
    ) -> Dict[str, str]:
        """
        Allocate multiple tasks to drones using globally optimal assignment.
        Uses Hungarian algorithm when strategy is OPTIMAL.
        
        Args:
            tasks: List of (task_id, requirements) tuples
            available_drones: List of available drones
            
        Returns:
            Dict mapping task_id to drone_id
        """
        if not tasks or not available_drones:
            return {}
        
        # Filter suitable drones for each task
        task_suitable_drones = {}
        for task_id, requirements in tasks:
            suitable = self._filter_suitable_drones(available_drones, requirements)
            task_suitable_drones[task_id] = suitable
        
        if self.strategy == AllocationStrategy.OPTIMAL and SCIPY_AVAILABLE:
            return self._optimal_batch_allocation(tasks, available_drones)
        else:
            # Fall back to sequential allocation
            allocations = {}
            remaining_drones = list(available_drones)
            
            for task_id, requirements in tasks:
                if not remaining_drones:
                    break
                    
                suitable = self._filter_suitable_drones(remaining_drones, requirements)
                if suitable:
                    selected = self._nearest_allocation(suitable, requirements)
                    if selected:
                        allocations[task_id] = selected.drone_id
                        remaining_drones = [d for d in remaining_drones 
                                          if d.drone_id != selected.drone_id]
            
            return allocations
    
    def _optimal_batch_allocation(
        self,
        tasks: List[Tuple[str, TaskRequirements]],
        drones: List[DroneInfo]
    ) -> Dict[str, str]:
        """
        Use Hungarian algorithm for globally optimal task assignment.
        
        Minimizes total cost across all assignments where cost considers:
        - Distance to task location
        - Battery level (higher = lower cost)
        - Current workload (fewer tasks = lower cost)
        
        Args:
            tasks: List of (task_id, requirements) tuples
            drones: List of available drones
            
        Returns:
            Dict mapping task_id to drone_id
        """
        if not SCIPY_AVAILABLE:
            self.logger.warning("scipy not available for optimal allocation")
            return {}
        
        n_tasks = len(tasks)
        n_drones = len(drones)
        
        if n_tasks == 0 or n_drones == 0:
            return {}
        
        # Create cost matrix
        # Rows = drones, Columns = tasks
        # Use large cost (infinity) for infeasible assignments
        INF_COST = 1e9
        cost_matrix = np.full((n_drones, n_tasks), INF_COST)
        
        for i, drone in enumerate(drones):
            for j, (task_id, requirements) in enumerate(tasks):
                # Check if drone is suitable for this task
                if not self._is_drone_suitable(drone, requirements):
                    continue  # Leave as INF_COST
                
                # Calculate cost components
                distance_cost = self._calculate_distance_cost(drone, requirements)
                battery_cost = self._calculate_battery_cost(drone)
                load_cost = self._calculate_load_cost(drone)
                capability_cost = self._calculate_capability_cost(drone, requirements)
                
                # Weighted sum of costs
                total_cost = (
                    distance_cost * 1.0 +      # Weight for distance
                    battery_cost * 0.5 +        # Weight for battery
                    load_cost * 0.3 +           # Weight for load
                    capability_cost * 0.2       # Weight for capability match
                )
                
                cost_matrix[i, j] = total_cost
        
        # Handle case where we have more tasks than drones or vice versa
        if n_tasks > n_drones:
            # Pad with dummy drones (high cost)
            padding = np.full((n_tasks - n_drones, n_tasks), INF_COST)
            cost_matrix = np.vstack([cost_matrix, padding])
        elif n_drones > n_tasks:
            # Pad with dummy tasks (zero cost - no penalty for unused drones)
            padding = np.zeros((n_drones, n_drones - n_tasks))
            cost_matrix = np.hstack([cost_matrix, padding])
        
        # Solve assignment problem using Hungarian algorithm
        try:
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
        except Exception as e:
            self.logger.error(f"Optimal allocation failed: {e}")
            return {}
        
        # Extract valid assignments
        allocations = {}
        for row, col in zip(row_ind, col_ind):
            # Skip dummy assignments
            if row >= n_drones or col >= n_tasks:
                continue
            
            # Skip infeasible assignments
            if cost_matrix[row, col] >= INF_COST:
                continue
            
            task_id = tasks[col][0]
            drone_id = drones[row].drone_id
            
            allocations[task_id] = drone_id
            self.task_assignments[task_id] = drone_id
            self.drone_task_count[drone_id] = \
                self.drone_task_count.get(drone_id, 0) + 1
        
        self.logger.info(
            f"Optimal allocation: assigned {len(allocations)}/{n_tasks} tasks "
            f"to {len(set(allocations.values()))}/{n_drones} drones"
        )
        
        return allocations
    
    def _is_drone_suitable(self, drone: DroneInfo, requirements: TaskRequirements) -> bool:
        """Quick check if drone meets basic requirements"""
        if drone.battery_percentage < requirements.min_battery:
            return False
        if drone.battery_remaining_time < requirements.min_flight_time:
            return False
        if requirements.requires_camera and not drone.capabilities.has_camera:
            return False
        if requirements.requires_lidar and not drone.capabilities.has_lidar:
            return False
        if requirements.requires_gripper and not drone.capabilities.has_gripper:
            return False
        return True
    
    def _calculate_distance_cost(
        self,
        drone: DroneInfo,
        requirements: TaskRequirements
    ) -> float:
        """Calculate distance cost for assignment"""
        if not requirements.target_location:
            return 0.0
        
        dx = requirements.target_location[0] - drone.position_x
        dy = requirements.target_location[1] - drone.position_y
        dz = requirements.target_location[2] - drone.position_z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def _calculate_battery_cost(self, drone: DroneInfo) -> float:
        """Calculate battery cost (lower battery = higher cost)"""
        return 100.0 - drone.battery_percentage
    
    def _calculate_load_cost(self, drone: DroneInfo) -> float:
        """Calculate load cost (more current tasks = higher cost)"""
        task_count = self.drone_task_count.get(drone.drone_id, 0)
        return task_count * 20.0
    
    def _calculate_capability_cost(
        self,
        drone: DroneInfo,
        requirements: TaskRequirements
    ) -> float:
        """Calculate capability match cost"""
        cost = 0.0
        
        # Penalize overqualified drones (save specialized for specialized tasks)
        if drone.capabilities.has_lidar and not requirements.requires_lidar:
            cost += 5.0
        if drone.capabilities.has_gripper and not requirements.requires_gripper:
            cost += 5.0
        
        return cost
    
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
