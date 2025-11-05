#!/usr/bin/env python3
"""
Mission Queue - Priority-based mission queue for Squadron Manager
"""

import heapq
import time
from typing import Optional, Dict, List
from dataclasses import dataclass, field


@dataclass(order=True)
class PrioritizedMission:
    """
    Mission with priority for queue management.
    Uses dataclass ordering for heapq.
    """
    # Priority fields (used for ordering)
    priority: int = field(compare=True)  # Negative for max-heap behavior
    timestamp: float = field(compare=True)  # Tie-breaker: earlier first
    
    # Mission data (not used for ordering)
    mission_id: str = field(compare=False)
    mission_data: Dict = field(compare=False, default_factory=dict)
    retry_count: int = field(compare=False, default=0)
    
    def __post_init__(self):
        """Negate priority for max-heap behavior (highest priority first)"""
        self.priority = -abs(self.priority)


class MissionQueue:
    """
    Priority queue for mission management.
    Supports priority-based scheduling with retry tracking.
    """
    
    def __init__(self, logger=None):
        """Initialize mission queue"""
        self.logger = logger
        self.queue: List[PrioritizedMission] = []
        self.pending_missions: Dict[str, PrioritizedMission] = {}
        
    def add_mission(
        self,
        mission_id: str,
        mission_data: Dict,
        priority: int = 100
    ) -> bool:
        """
        Add mission to queue.
        
        Args:
            mission_id: Unique mission identifier
            mission_data: Mission parameters
            priority: Mission priority (higher = more important)
            
        Returns:
            True if added successfully
        """
        if mission_id in self.pending_missions:
            if self.logger:
                self.logger.warning(f"Mission {mission_id} already in queue")
            return False
        
        prioritized_mission = PrioritizedMission(
            priority=priority,
            timestamp=time.time(),
            mission_id=mission_id,
            mission_data=mission_data
        )
        
        heapq.heappush(self.queue, prioritized_mission)
        self.pending_missions[mission_id] = prioritized_mission
        
        if self.logger:
            self.logger.info(
                f"Mission {mission_id} added to queue "
                f"(priority={priority}, queue_size={len(self.queue)})"
            )
        
        return True
    
    def get_next_mission(self) -> Optional[PrioritizedMission]:
        """
        Get highest priority mission from queue.
        
        Returns:
            Next mission to execute, or None if queue empty
        """
        if not self.queue:
            return None
        
        mission = heapq.heappop(self.queue)
        
        # Remove from pending
        if mission.mission_id in self.pending_missions:
            del self.pending_missions[mission.mission_id]
        
        if self.logger:
            self.logger.info(
                f"Retrieved mission {mission.mission_id} from queue "
                f"(priority={-mission.priority}, queue_remaining={len(self.queue)})"
            )
        
        return mission
    
    def peek_next_mission(self) -> Optional[PrioritizedMission]:
        """
        Peek at highest priority mission without removing it.
        
        Returns:
            Next mission, or None if queue empty
        """
        if not self.queue:
            return None
        return self.queue[0]
    
    def remove_mission(self, mission_id: str) -> bool:
        """
        Remove specific mission from queue.
        
        Args:
            mission_id: Mission to remove
            
        Returns:
            True if removed successfully
        """
        if mission_id not in self.pending_missions:
            return False
        
        # Mark as removed (will be skipped on pop)
        mission = self.pending_missions[mission_id]
        del self.pending_missions[mission_id]
        
        # Rebuild heap without removed mission
        self.queue = [m for m in self.queue if m.mission_id != mission_id]
        heapq.heapify(self.queue)
        
        if self.logger:
            self.logger.info(f"Removed mission {mission_id} from queue")
        
        return True
    
    def requeue_mission_with_higher_priority(
        self,
        mission_id: str,
        priority_boost: int = 50
    ) -> bool:
        """
        Re-queue failed mission with higher priority.
        
        Args:
            mission_id: Mission to re-queue
            priority_boost: Amount to increase priority
            
        Returns:
            True if re-queued successfully
        """
        if mission_id not in self.pending_missions:
            if self.logger:
                self.logger.warning(
                    f"Cannot re-queue {mission_id}: not in pending missions"
                )
            return False
        
        # Get original mission
        old_mission = self.pending_missions[mission_id]
        
        # Remove old entry
        self.remove_mission(mission_id)
        
        # Add with higher priority and incremented retry count
        new_priority = -old_mission.priority + priority_boost
        
        new_mission = PrioritizedMission(
            priority=new_priority,
            timestamp=time.time(),
            mission_id=mission_id,
            mission_data=old_mission.mission_data,
            retry_count=old_mission.retry_count + 1
        )
        
        heapq.heappush(self.queue, new_mission)
        self.pending_missions[mission_id] = new_mission
        
        if self.logger:
            self.logger.info(
                f"Re-queued mission {mission_id} with priority {new_priority} "
                f"(retry #{new_mission.retry_count})"
            )
        
        return True
    
    def is_empty(self) -> bool:
        """Check if queue is empty"""
        return len(self.queue) == 0
    
    def size(self) -> int:
        """Get number of missions in queue"""
        return len(self.queue)
    
    def clear(self):
        """Clear all missions from queue"""
        self.queue.clear()
        self.pending_missions.clear()
        
        if self.logger:
            self.logger.info("Mission queue cleared")
    
    def get_pending_mission_ids(self) -> List[str]:
        """Get list of all pending mission IDs"""
        return list(self.pending_missions.keys())
    
    def get_queue_status(self) -> Dict:
        """
        Get queue status information.
        
        Returns:
            Dictionary with queue statistics
        """
        return {
            'size': len(self.queue),
            'pending_missions': len(self.pending_missions),
            'mission_ids': list(self.pending_missions.keys()),
            'next_mission_id': self.peek_next_mission().mission_id if self.queue else None,
            'next_priority': -self.peek_next_mission().priority if self.queue else None
        }
