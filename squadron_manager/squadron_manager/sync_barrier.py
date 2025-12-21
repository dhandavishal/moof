#!/usr/bin/env python3
"""
Synchronization Barrier - Coordination sync points for multi-drone operations
"""

import asyncio
import time
from typing import Dict, Set, Optional, Callable, Any
from dataclasses import dataclass, field
from enum import Enum
import threading


class BarrierState(Enum):
    """Barrier states"""
    WAITING = "waiting"
    COMPLETE = "complete"
    TIMEOUT = "timeout"
    CANCELLED = "cancelled"


@dataclass
class DroneArrival:
    """Information about a drone arriving at barrier"""
    drone_id: str
    arrival_time: float
    data: Dict = field(default_factory=dict)


class SyncBarrier:
    """
    Synchronization point for multi-drone coordination.
    
    Allows drones to wait at a barrier until all expected drones arrive,
    enabling coordinated operations like simultaneous takeoff, formation
    assembly, or synchronized survey start.
    
    Example usage:
        # Create barrier for 3 drones with 30 second timeout
        barrier = SyncBarrier("takeoff", num_drones=3, timeout=30.0)
        
        # Each drone calls arrive when ready
        barrier.arrive("drone_0", {"position": (0, 0, 50)})
        barrier.arrive("drone_1", {"position": (10, 0, 50)})
        barrier.arrive("drone_2", {"position": (20, 0, 50)})
        
        # Check if all arrived
        if barrier.is_complete():
            # All drones ready, proceed with coordinated action
            pass
    """
    
    def __init__(
        self,
        name: str,
        num_drones: int,
        timeout: float = 30.0,
        on_complete: Optional[Callable[[Dict[str, DroneArrival]], None]] = None,
        on_timeout: Optional[Callable[[Set[str], Set[str]], None]] = None,
        logger=None
    ):
        """
        Initialize synchronization barrier.
        
        Args:
            name: Barrier name (for logging)
            num_drones: Number of drones expected to arrive
            timeout: Maximum time to wait for all drones (seconds)
            on_complete: Callback when all drones arrive
            on_timeout: Callback on timeout (receives arrived, missing)
            logger: Logger instance
        """
        self.name = name
        self.expected_count = num_drones
        self.timeout = timeout
        self.on_complete = on_complete
        self.on_timeout = on_timeout
        self.logger = logger
        
        # State
        self.arrivals: Dict[str, DroneArrival] = {}
        self.expected_drones: Set[str] = set()
        self.state = BarrierState.WAITING
        self.creation_time = time.time()
        self._lock = threading.Lock()
        
        # Async event for waiting
        self._complete_event = threading.Event()
    
    def set_expected_drones(self, drone_ids: Set[str]):
        """
        Set specific drone IDs expected at this barrier.
        
        Args:
            drone_ids: Set of drone IDs expected to arrive
        """
        with self._lock:
            self.expected_drones = drone_ids
            self.expected_count = len(drone_ids)
            
            if self.logger:
                self.logger.info(
                    f"Barrier '{self.name}': Expecting drones {drone_ids}"
                )
    
    def arrive(self, drone_id: str, data: Optional[Dict] = None) -> bool:
        """
        Mark a drone as arrived at the barrier.
        
        Args:
            drone_id: ID of arriving drone
            data: Optional data to associate with arrival
            
        Returns:
            True if arrival recorded, False if barrier already complete/cancelled
        """
        with self._lock:
            if self.state != BarrierState.WAITING:
                if self.logger:
                    self.logger.warning(
                        f"Barrier '{self.name}': Drone {drone_id} arrived but "
                        f"barrier already in state {self.state.value}"
                    )
                return False
            
            # Record arrival
            arrival = DroneArrival(
                drone_id=drone_id,
                arrival_time=time.time(),
                data=data or {}
            )
            self.arrivals[drone_id] = arrival
            
            if self.logger:
                self.logger.info(
                    f"Barrier '{self.name}': Drone {drone_id} arrived "
                    f"({len(self.arrivals)}/{self.expected_count})"
                )
            
            # Check if all expected drones have arrived
            if len(self.arrivals) >= self.expected_count:
                self._complete()
            
            return True
    
    def depart(self, drone_id: str) -> bool:
        """
        Remove a drone from the barrier (e.g., if it failed).
        
        Args:
            drone_id: ID of departing drone
            
        Returns:
            True if removed, False if not present
        """
        with self._lock:
            if drone_id in self.arrivals:
                del self.arrivals[drone_id]
                
                if self.logger:
                    self.logger.warning(
                        f"Barrier '{self.name}': Drone {drone_id} departed "
                        f"({len(self.arrivals)}/{self.expected_count})"
                    )
                return True
            return False
    
    def reduce_expected(self, new_count: int):
        """
        Reduce expected drone count (e.g., if a drone failed).
        
        Args:
            new_count: New expected count
        """
        with self._lock:
            old_count = self.expected_count
            self.expected_count = new_count
            
            if self.logger:
                self.logger.info(
                    f"Barrier '{self.name}': Reduced expected from "
                    f"{old_count} to {new_count}"
                )
            
            # Check if we now have enough
            if len(self.arrivals) >= self.expected_count:
                self._complete()
    
    def _complete(self):
        """Mark barrier as complete and trigger callback"""
        self.state = BarrierState.COMPLETE
        self._complete_event.set()
        
        if self.logger:
            elapsed = time.time() - self.creation_time
            self.logger.info(
                f"Barrier '{self.name}': Complete! "
                f"All {self.expected_count} drones arrived in {elapsed:.2f}s"
            )
        
        if self.on_complete:
            self.on_complete(self.arrivals)
    
    def is_complete(self) -> bool:
        """Check if barrier is complete"""
        return self.state == BarrierState.COMPLETE
    
    def is_waiting(self) -> bool:
        """Check if barrier is still waiting"""
        return self.state == BarrierState.WAITING
    
    def get_arrived(self) -> Set[str]:
        """Get set of drone IDs that have arrived"""
        with self._lock:
            return set(self.arrivals.keys())
    
    def get_missing(self) -> Set[str]:
        """Get set of expected drone IDs that haven't arrived"""
        with self._lock:
            if self.expected_drones:
                return self.expected_drones - set(self.arrivals.keys())
            return set()
    
    def get_progress(self) -> float:
        """Get progress as percentage (0-100)"""
        with self._lock:
            if self.expected_count == 0:
                return 100.0
            return (len(self.arrivals) / self.expected_count) * 100.0
    
    def wait(self, timeout: Optional[float] = None) -> BarrierState:
        """
        Blocking wait for barrier completion.
        
        Args:
            timeout: Override default timeout (seconds)
            
        Returns:
            Final barrier state
        """
        wait_timeout = timeout if timeout is not None else self.timeout
        
        if self._complete_event.wait(timeout=wait_timeout):
            return self.state
        
        # Timeout occurred
        with self._lock:
            if self.state == BarrierState.WAITING:
                self.state = BarrierState.TIMEOUT
                
                if self.logger:
                    arrived = set(self.arrivals.keys())
                    missing = self.expected_drones - arrived if self.expected_drones else set()
                    self.logger.error(
                        f"Barrier '{self.name}': TIMEOUT! "
                        f"Only {len(self.arrivals)}/{self.expected_count} arrived. "
                        f"Missing: {missing}"
                    )
                
                if self.on_timeout:
                    arrived = set(self.arrivals.keys())
                    missing = self.expected_drones - arrived if self.expected_drones else set()
                    self.on_timeout(arrived, missing)
        
        return self.state
    
    async def wait_async(self, timeout: Optional[float] = None) -> BarrierState:
        """
        Async wait for barrier completion.
        
        Args:
            timeout: Override default timeout (seconds)
            
        Returns:
            Final barrier state
        """
        wait_timeout = timeout if timeout is not None else self.timeout
        start_time = time.time()
        
        while self.state == BarrierState.WAITING:
            if time.time() - start_time > wait_timeout:
                # Timeout
                with self._lock:
                    if self.state == BarrierState.WAITING:
                        self.state = BarrierState.TIMEOUT
                        
                        if self.logger:
                            arrived = set(self.arrivals.keys())
                            missing = self.expected_drones - arrived if self.expected_drones else set()
                            self.logger.error(
                                f"Barrier '{self.name}': TIMEOUT! "
                                f"Only {len(self.arrivals)}/{self.expected_count} arrived. "
                                f"Missing: {missing}"
                            )
                        
                        if self.on_timeout:
                            arrived = set(self.arrivals.keys())
                            missing = self.expected_drones - arrived
                            self.on_timeout(arrived, missing)
                break
            
            await asyncio.sleep(0.1)
        
        return self.state
    
    def cancel(self):
        """Cancel the barrier"""
        with self._lock:
            self.state = BarrierState.CANCELLED
            self._complete_event.set()
            
            if self.logger:
                self.logger.warning(f"Barrier '{self.name}': Cancelled")
    
    def reset(self):
        """Reset barrier for reuse"""
        with self._lock:
            self.arrivals.clear()
            self.state = BarrierState.WAITING
            self.creation_time = time.time()
            self._complete_event.clear()
            
            if self.logger:
                self.logger.info(f"Barrier '{self.name}': Reset")
    
    def get_status(self) -> Dict:
        """Get barrier status summary"""
        with self._lock:
            return {
                'name': self.name,
                'state': self.state.value,
                'expected': self.expected_count,
                'arrived': len(self.arrivals),
                'arrived_drones': list(self.arrivals.keys()),
                'missing_drones': list(self.get_missing()),
                'progress': self.get_progress(),
                'elapsed': time.time() - self.creation_time,
                'timeout': self.timeout
            }


class BarrierManager:
    """
    Manages multiple synchronization barriers for squadron coordination.
    
    Common barriers:
    - takeoff_barrier: All drones have taken off and reached hover altitude
    - formation_barrier: All drones in position for formation
    - waypoint_barrier: All drones reached specific waypoint
    - landing_barrier: All drones ready for coordinated landing
    """
    
    def __init__(self, logger=None):
        """Initialize barrier manager"""
        self.logger = logger
        self.barriers: Dict[str, SyncBarrier] = {}
        self._lock = threading.Lock()
    
    def create_barrier(
        self,
        name: str,
        num_drones: int,
        timeout: float = 30.0,
        on_complete: Optional[Callable] = None,
        on_timeout: Optional[Callable] = None
    ) -> SyncBarrier:
        """
        Create a new synchronization barrier.
        
        Args:
            name: Unique barrier name
            num_drones: Expected drone count
            timeout: Barrier timeout
            on_complete: Completion callback
            on_timeout: Timeout callback
            
        Returns:
            Created barrier
        """
        with self._lock:
            if name in self.barriers:
                if self.logger:
                    self.logger.warning(
                        f"Barrier '{name}' already exists, replacing"
                    )
            
            barrier = SyncBarrier(
                name=name,
                num_drones=num_drones,
                timeout=timeout,
                on_complete=on_complete,
                on_timeout=on_timeout,
                logger=self.logger
            )
            
            self.barriers[name] = barrier
            
            if self.logger:
                self.logger.info(
                    f"Created barrier '{name}' for {num_drones} drones "
                    f"(timeout: {timeout}s)"
                )
            
            return barrier
    
    def get_barrier(self, name: str) -> Optional[SyncBarrier]:
        """Get barrier by name"""
        return self.barriers.get(name)
    
    def remove_barrier(self, name: str) -> bool:
        """Remove a barrier"""
        with self._lock:
            if name in self.barriers:
                del self.barriers[name]
                
                if self.logger:
                    self.logger.info(f"Removed barrier '{name}'")
                return True
            return False
    
    def arrive_at_barrier(
        self,
        barrier_name: str,
        drone_id: str,
        data: Optional[Dict] = None
    ) -> bool:
        """
        Mark drone as arrived at a specific barrier.
        
        Args:
            barrier_name: Name of barrier
            drone_id: Arriving drone ID
            data: Optional arrival data
            
        Returns:
            True if recorded, False if barrier doesn't exist
        """
        barrier = self.barriers.get(barrier_name)
        if barrier:
            return barrier.arrive(drone_id, data)
        
        if self.logger:
            self.logger.warning(
                f"Drone {drone_id} tried to arrive at non-existent "
                f"barrier '{barrier_name}'"
            )
        return False
    
    def handle_drone_failure(self, drone_id: str):
        """
        Handle drone failure across all active barriers.
        
        Args:
            drone_id: Failed drone ID
        """
        with self._lock:
            for name, barrier in self.barriers.items():
                if barrier.is_waiting():
                    # Remove drone from barrier
                    barrier.depart(drone_id)
                    
                    # Reduce expected count
                    if barrier.expected_count > 1:
                        barrier.reduce_expected(barrier.expected_count - 1)
                    
                    if self.logger:
                        self.logger.warning(
                            f"Drone {drone_id} failed - removed from barrier '{name}'"
                        )
    
    def get_all_status(self) -> Dict[str, Dict]:
        """Get status of all barriers"""
        with self._lock:
            return {
                name: barrier.get_status()
                for name, barrier in self.barriers.items()
            }
    
    def clear_completed(self):
        """Remove all completed barriers"""
        with self._lock:
            completed = [
                name for name, barrier in self.barriers.items()
                if barrier.state in [BarrierState.COMPLETE, BarrierState.CANCELLED]
            ]
            
            for name in completed:
                del self.barriers[name]
            
            if completed and self.logger:
                self.logger.info(f"Cleared {len(completed)} completed barriers")
