#!/usr/bin/env python3
"""
Drone Registry - Tracks all drones in the squadron
"""

from typing import Dict, List, Optional
from dataclasses import dataclass, field
from enum import Enum
import time


class DroneState(Enum):
    """Drone operational states"""
    UNKNOWN = 0
    AVAILABLE = 1
    BUSY = 2
    CHARGING = 3
    MAINTENANCE = 4
    ERROR = 5
    OFFLINE = 6


@dataclass
class DroneCapabilities:
    """Capabilities of a drone"""
    max_speed: float = 5.0  # m/s
    max_altitude: float = 100.0  # meters
    flight_time: float = 20.0  # minutes
    payload_capacity: float = 0.5  # kg
    has_camera: bool = True
    has_lidar: bool = False
    has_gripper: bool = False


@dataclass
class DroneInfo:
    """Complete information about a drone"""
    drone_id: str
    namespace: str
    state: DroneState = DroneState.UNKNOWN
    capabilities: DroneCapabilities = field(default_factory=DroneCapabilities)
    
    # Status
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    
    # Position (local NED)
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    
    # Battery
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    battery_remaining_time: float = 0.0  # minutes
    
    # GPS
    gps_fix_type: int = 0
    gps_satellites: int = 0
    
    # Task assignment
    current_task_id: Optional[str] = None
    task_progress: float = 0.0
    
    # Health
    last_heartbeat: float = 0.0
    health_status: str = "UNKNOWN"
    error_messages: List[str] = field(default_factory=list)
    
    def is_available(self) -> bool:
        """Check if drone is available for new tasks"""
        return (
            self.state == DroneState.AVAILABLE and
            self.connected and
            self.battery_percentage > 20.0 and
            self.gps_fix_type >= 2 and
            self.current_task_id is None
        )
    
    def is_healthy(self) -> bool:
        """Check if drone is healthy"""
        now = time.time()
        return (
            self.connected and
            (now - self.last_heartbeat) < 5.0 and  # 5 second timeout
            self.state not in [DroneState.ERROR, DroneState.OFFLINE] and
            len(self.error_messages) == 0
        )
    
    def get_estimated_range(self) -> float:
        """Estimate remaining flight range in meters"""
        # Simple estimation: battery time * average speed
        avg_speed = self.capabilities.max_speed * 0.6  # 60% of max
        return self.battery_remaining_time * 60 * avg_speed  # minutes to seconds


class DroneRegistry:
    """
    Central registry for all drones in the squadron.
    Tracks status, capabilities, and availability.
    """
    
    def __init__(self, logger):
        """Initialize drone registry"""
        self.logger = logger
        self.drones: Dict[str, DroneInfo] = {}
        
    def register_drone(
        self,
        drone_id: str,
        namespace: str,
        capabilities: Optional[DroneCapabilities] = None
    ) -> bool:
        """
        Register a new drone in the squadron.
        
        Args:
            drone_id: Unique identifier for the drone
            namespace: ROS2 namespace for the drone (e.g., '/drone_0')
            capabilities: Drone capabilities
            
        Returns:
            True if registration successful
        """
        if drone_id in self.drones:
            self.logger.warning(f"Drone {drone_id} already registered")
            return False
        
        if capabilities is None:
            capabilities = DroneCapabilities()
        
        drone_info = DroneInfo(
            drone_id=drone_id,
            namespace=namespace,
            capabilities=capabilities,
            last_heartbeat=time.time()
        )
        
        self.drones[drone_id] = drone_info
        self.logger.info(f"Registered drone: {drone_id} (namespace: {namespace})")
        return True
    
    def unregister_drone(self, drone_id: str) -> bool:
        """Unregister a drone from the squadron"""
        if drone_id not in self.drones:
            self.logger.warning(f"Drone {drone_id} not found in registry")
            return False
        
        del self.drones[drone_id]
        self.logger.info(f"Unregistered drone: {drone_id}")
        return True
    
    def get_drone(self, drone_id: str) -> Optional[DroneInfo]:
        """Get drone information by ID"""
        return self.drones.get(drone_id)
    
    def get_all_drones(self) -> List[DroneInfo]:
        """Get list of all registered drones"""
        return list(self.drones.values())
    
    def get_available_drones(self) -> List[DroneInfo]:
        """Get list of available drones ready for tasks"""
        return [drone for drone in self.drones.values() if drone.is_available()]
    
    def get_busy_drones(self) -> List[DroneInfo]:
        """Get list of drones currently executing tasks"""
        return [drone for drone in self.drones.values() if drone.state == DroneState.BUSY]
    
    def get_unhealthy_drones(self) -> List[DroneInfo]:
        """Get list of drones with health issues"""
        return [drone for drone in self.drones.values() if not drone.is_healthy()]
    
    def update_drone_state(self, drone_id: str, state: DroneState) -> bool:
        """Update drone operational state"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        drone.state = state
        return True
    
    def update_drone_position(self, drone_id: str, x: float, y: float, z: float) -> bool:
        """Update drone position"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        drone.position_x = x
        drone.position_y = y
        drone.position_z = z
        return True
    
    def update_drone_battery(
        self,
        drone_id: str,
        voltage: float,
        percentage: float,
        remaining_time: float
    ) -> bool:
        """Update drone battery status"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        drone.battery_voltage = voltage
        drone.battery_percentage = percentage
        drone.battery_remaining_time = remaining_time
        return True
    
    def update_drone_gps(self, drone_id: str, fix_type: int, satellites: int) -> bool:
        """Update drone GPS status"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        drone.gps_fix_type = fix_type
        drone.gps_satellites = satellites
        return True
    
    def update_drone_connection(
        self,
        drone_id: str,
        connected: bool,
        armed: bool = False,
        mode: str = "UNKNOWN"
    ) -> bool:
        """Update drone connection status"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        old_state = drone.state
        old_connected = drone.connected
        
        drone.connected = connected
        drone.armed = armed
        drone.mode = mode
        drone.last_heartbeat = time.time()
        
        # Update state based on connection and current state
        if not connected:
            drone.state = DroneState.OFFLINE
        elif drone.state in [DroneState.UNKNOWN, DroneState.OFFLINE]:
            # Drone just connected or reconnected - mark as available if no task
            if drone.current_task_id is None:
                drone.state = DroneState.AVAILABLE
            else:
                drone.state = DroneState.BUSY
        
        # Log state transitions
        if old_state != drone.state or old_connected != drone.connected:
            self.logger.info(
                f"Drone {drone_id} updated: "
                f"connected={old_connected}->{connected}, "
                f"state={old_state.name}->{drone.state.name}, "
                f"armed={armed}, mode={mode}"
            )
        
        return True
    
    def assign_task(self, drone_id: str, task_id: str) -> bool:
        """Assign a task to a drone"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        if not drone.is_available():
            self.logger.warning(f"Cannot assign task to {drone_id}: drone not available")
            return False
        
        drone.current_task_id = task_id
        drone.task_progress = 0.0
        drone.state = DroneState.BUSY
        self.logger.info(f"Assigned task {task_id} to drone {drone_id}")
        return True
    
    def complete_task(self, drone_id: str) -> bool:
        """Mark task as completed for a drone"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        task_id = drone.current_task_id
        drone.current_task_id = None
        drone.task_progress = 0.0
        drone.state = DroneState.AVAILABLE
        self.logger.info(f"Task {task_id} completed by drone {drone_id}")
        return True
    
    def update_task_progress(self, drone_id: str, progress: float) -> bool:
        """Update task progress for a drone"""
        drone = self.get_drone(drone_id)
        if drone is None:
            return False
        
        drone.task_progress = max(0.0, min(100.0, progress))
        return True
    
    def get_squadron_status(self) -> Dict:
        """Get overall squadron status"""
        total = len(self.drones)
        available = len(self.get_available_drones())
        busy = len(self.get_busy_drones())
        unhealthy = len(self.get_unhealthy_drones())
        
        return {
            'total_drones': total,
            'available': available,
            'busy': busy,
            'unhealthy': unhealthy,
            'average_battery': sum(d.battery_percentage for d in self.drones.values()) / max(total, 1)
        }
