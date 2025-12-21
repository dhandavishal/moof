"""
Squadron Manager - Multi-drone coordination package

Modules:
- drone_registry: Drone registration and status tracking
- task_allocator: Task allocation with various strategies including Hungarian algorithm
- formation_controller: Formation flying coordination
- sync_barrier: Synchronization barriers for coordinated operations
- collision_avoidance: Inter-drone collision avoidance
- squadron_manager_node: Main ROS2 node for squadron coordination
"""

from squadron_manager.drone_registry import DroneRegistry, DroneInfo, DroneState, DroneCapabilities
from squadron_manager.task_allocator import TaskAllocator, AllocationStrategy, TaskRequirements
from squadron_manager.formation_controller import FormationController, FormationType, FormationParameters
from squadron_manager.sync_barrier import SyncBarrier, BarrierManager, BarrierState

__all__ = [
    'DroneRegistry',
    'DroneInfo',
    'DroneState',
    'DroneCapabilities',
    'TaskAllocator',
    'AllocationStrategy',
    'TaskRequirements',
    'FormationController',
    'FormationType',
    'FormationParameters',
    'SyncBarrier',
    'BarrierManager',
    'BarrierState',
]