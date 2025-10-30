#!/usr/bin/env python3
"""
Priority-based task queue with dependency resolution for ROS2.

Uses heapq for priority ordering and NetworkX for dependency graph management.
"""

import heapq
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Any
from enum import Enum

from rclpy.node import Node

try:
    import networkx as nx
    NETWORKX_AVAILABLE = True
except ImportError:
    NETWORKX_AVAILABLE = False


class TaskStatus(Enum):
    """Task execution status"""
    QUEUED = "queued"
    ACTIVE = "active"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(order=True)
class PrioritizedTask:
    """
    Task with priority ordering for heap queue.
    Lower priority value = higher priority (executes first).
    """
    priority: int
    task_id: str = field(compare=False)
    task_type: str = field(compare=False)
    parameters: Dict[str, Any] = field(compare=False)
    dependencies: List[str] = field(default_factory=list, compare=False)
    timeout: float = field(default=300.0, compare=False)  # seconds
    status: TaskStatus = field(default=TaskStatus.QUEUED, compare=False)
    
    def to_dict(self):
        """Convert to dictionary for serialization"""
        return {
            'task_id': self.task_id,
            'task_type': self.task_type,
            'priority': self.priority,
            'parameters': self.parameters,
            'dependencies': self.dependencies,
            'timeout': self.timeout,
            'status': self.status.value
        }


class TaskQueue:
    """
    Priority-based task queue with dependency resolution.
    """
    
    # Priority level definitions
    PRIORITY_EMERGENCY = 0
    PRIORITY_CRITICAL = 10
    PRIORITY_HIGH = 20
    PRIORITY_NORMAL = 50
    PRIORITY_LOW = 100
    
    def __init__(self, node: Node):
        """
        Initialize task queue.
        
        Args:
            node: ROS2 node instance
        """
        self.node = node
        self.queue: List[PrioritizedTask] = []
        self.active_task: Optional[PrioritizedTask] = None
        self.task_registry: Dict[str, PrioritizedTask] = {}
        self.completed_tasks: Dict[str, Dict] = {}
        self.failed_tasks: Dict[str, Dict] = {}
        self.paused_tasks: Dict[str, Dict] = {}
        
        # Dependency graph for task ordering
        if NETWORKX_AVAILABLE:
            self.dependency_graph = nx.DiGraph()
        else:
            self.node.get_logger().warn(
                "NetworkX not available - dependency resolution disabled"
            )
            self.dependency_graph = None
        
        self.node.get_logger().info("Task queue initialized")
    
    def enqueue(self, task: PrioritizedTask) -> bool:
        """
        Add task to queue with validation.
        
        Args:
            task: Task to enqueue
            
        Returns:
            True if successfully queued, False otherwise
        """
        # Check for duplicate task ID
        if task.task_id in self.task_registry:
            self.node.get_logger().warn(f"Task {task.task_id} already exists in queue")
            return False
        
        # Validate dependencies if NetworkX available
        if self.dependency_graph is not None and task.dependencies:
            for dep_id in task.dependencies:
                if dep_id not in self.task_registry:
                    self.node.get_logger().error(
                        f"Task {task.task_id} has unknown dependency: {dep_id}"
                    )
                    return False
                
                # Add edge to dependency graph
                self.dependency_graph.add_edge(dep_id, task.task_id)
            
            # Check for circular dependencies
            if not nx.is_directed_acyclic_graph(self.dependency_graph):
                self.node.get_logger().error(
                    f"Task {task.task_id} creates circular dependency"
                )
                # Remove the edges we just added
                for dep_id in task.dependencies:
                    self.dependency_graph.remove_edge(dep_id, task.task_id)
                return False
        
        # Add to queue
        heapq.heappush(self.queue, task)
        self.task_registry[task.task_id] = task
        
        self.node.get_logger().info(
            f"Task {task.task_id} ({task.task_type}) enqueued with priority {task.priority}"
        )
        
        return True
    
    def get_next_task(self) -> Optional[PrioritizedTask]:
        """
        Get next task with satisfied dependencies.
        
        Returns:
            Next executable task or None if queue empty or dependencies not met
        """
        attempts = 0
        max_attempts = len(self.queue)
        
        while self.queue and attempts < max_attempts:
            # Pop highest priority task
            task = heapq.heappop(self.queue)
            
            # Check dependencies
            if self._dependencies_satisfied(task):
                self.active_task = task
                task.status = TaskStatus.ACTIVE
                self.node.get_logger().info(f"Starting task: {task.task_id}")
                return task
            else:
                # Dependencies not met - re-queue
                heapq.heappush(self.queue, task)
                attempts += 1
        
        if attempts >= max_attempts:
            self.node.get_logger().warn("Potential dependency deadlock detected")
        
        return None
    
    def _dependencies_satisfied(self, task: PrioritizedTask) -> bool:
        """Check if all task dependencies are satisfied"""
        for dep_id in task.dependencies:
            # Check if dependency completed successfully
            if dep_id not in self.completed_tasks:
                self.node.get_logger().debug(
                    f"Task {task.task_id} waiting for dependency {dep_id}"
                )
                return False
            
            if self.completed_tasks[dep_id]['status'] != TaskStatus.COMPLETED:
                self.node.get_logger().error(
                    f"Dependency {dep_id} failed - cannot execute {task.task_id}"
                )
                task.status = TaskStatus.FAILED
                self.failed_tasks[task.task_id] = {
                    'task': task,
                    'reason': 'dependency_failure',
                    'failed_dependency': dep_id,
                    'timestamp': self.node.get_clock().now().nanoseconds / 1e9
                }
                return False
        
        return True
    
    def mark_completed(self, task_id: str, result: Optional[Dict] = None):
        """Mark task as completed"""
        if task_id not in self.task_registry:
            self.node.get_logger().warn(f"Cannot mark unknown task {task_id} as completed")
            return
        
        task = self.task_registry[task_id]
        task.status = TaskStatus.COMPLETED
        
        self.completed_tasks[task_id] = {
            'task': task,
            'status': TaskStatus.COMPLETED,
            'result': result,
            'timestamp': self.node.get_clock().now().nanoseconds / 1e9
        }
        
        # Clear active task if this was it
        if self.active_task and self.active_task.task_id == task_id:
            self.active_task = None
        
        self.node.get_logger().info(f"Task {task_id} marked as completed")
    
    def mark_failed(self, task_id: str, reason: str):
        """Mark task as failed"""
        if task_id not in self.task_registry:
            self.node.get_logger().warn(f"Cannot mark unknown task {task_id} as failed")
            return
        
        task = self.task_registry[task_id]
        task.status = TaskStatus.FAILED
        
        self.failed_tasks[task_id] = {
            'task': task,
            'reason': reason,
            'timestamp': self.node.get_clock().now().nanoseconds / 1e9
        }
        
        # Clear active task
        if self.active_task and self.active_task.task_id == task_id:
            self.active_task = None
        
        self.node.get_logger().error(f"Task {task_id} marked as failed: {reason}")
    
    def inject_emergency_task(self, task: PrioritizedTask):
        """
        Insert emergency task at front of queue, pausing current task.
        
        Args:
            task: Emergency task (will be forced to PRIORITY_EMERGENCY)
        """
        # Force emergency priority
        task.priority = self.PRIORITY_EMERGENCY
        
        # Pause current task if executing
        if self.active_task:
            self.node.get_logger().warn(
                f"Pausing task {self.active_task.task_id} for emergency: {task.task_id}"
            )
            
            self.paused_tasks[self.active_task.task_id] = {
                'task': self.active_task,
                'timestamp': self.node.get_clock().now().nanoseconds / 1e9
            }
            
            self.active_task = None
        
        # Enqueue emergency task
        self.enqueue(task)
    
    def has_tasks(self) -> bool:
        """Check if queue has any tasks"""
        return len(self.queue) > 0
    
    def peek(self) -> Optional[PrioritizedTask]:
        """Look at highest priority task without removing it"""
        if self.queue:
            return self.queue[0]
        return None
    
    def clear_queue(self):
        """Remove all queued tasks (emergency use)"""
        self.node.get_logger().warn("Clearing all queued tasks")
        self.queue.clear()
    
    def get_queue_status(self) -> Dict:
        """Get current queue status"""
        return {
            'queued_count': len(self.queue),
            'active_task': self.active_task.task_id if self.active_task else None,
            'completed_count': len(self.completed_tasks),
            'failed_count': len(self.failed_tasks),
            'paused_count': len(self.paused_tasks)
        }
