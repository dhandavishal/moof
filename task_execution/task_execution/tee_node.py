#!/usr/bin/env python3
"""
Main Task Execution Engine node for ROS2.
Orchestrates mission execution from high-level commands to low-level primitives.
"""

import json
import time
from typing import Optional, Dict, List
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Core components
from task_execution.core.state_machine import StateMachine, MissionState
from task_execution.core.task_queue import TaskQueue, PrioritizedTask, TaskStatus
from task_execution.core.task_validator import TaskValidator

# Executors
from task_execution.executors.waypoint_executor import WaypointExecutor
from task_execution.executors.survey_executor import SurveyExecutor
from task_execution.executors.search_executor import SearchExecutor

# Monitors
from task_execution.monitors.battery_monitor import BatteryMonitor
from task_execution.monitors.gps_monitor import GPSMonitor
from task_execution.monitors.health_monitor import HealthMonitor
from task_execution.monitors.progress_monitor import ProgressMonitor

# Messages
from multi_drone_msgs.msg import PrimitiveCommand, PrimitiveStatus
from multi_drone_msgs.action import Takeoff, Land, GoToWaypoint, ExecutePrimitive
from multi_drone_msgs.srv import ArmDisarm
from std_msgs.msg import Empty, String
from rclpy.action import ActionClient


class TaskExecutionEngineNode(Node):
    """
    Main Task Execution Engine node.
    Orchestrates mission execution from high-level commands to low-level primitives.
    """
    
    def __init__(self):
        """Initialize TEE node with all components"""
        
        super().__init__('task_execution_engine')
        
        self.get_logger().info("Initializing Task Execution Engine...")
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Load configuration (simplified - using dict for now)
        self.config = self._load_default_config()
        
        # Initialize core components
        self.state_machine = StateMachine(self)
        self.task_queue = TaskQueue(self)
        self.validator = TaskValidator(self, self.config)
        
        # Initialize monitors
        self.battery_monitor = BatteryMonitor(self, self.config, namespace='/drone_0')
        self.gps_monitor = GPSMonitor(self, self.config, namespace='/drone_0')
        self.health_monitor = HealthMonitor(self, self.battery_monitor, self.gps_monitor)
        self.progress_monitor = ProgressMonitor(self)
        
        # Drone namespace
        self.drone_namespace = '/drone_0'  # TODO: Make configurable
        
        # FAL Action Clients - for action-based communication
        self.get_logger().info(f"Creating FAL action clients for {self.drone_namespace}")
        
        self.arm_client = self.create_client(
            ArmDisarm,
            f'{self.drone_namespace}/arm_disarm'
        )
        
        self.takeoff_action_client = ActionClient(
            self,
            Takeoff,
            f'{self.drone_namespace}/takeoff',
            callback_group=self.callback_group
        )
        
        self.land_action_client = ActionClient(
            self,
            Land,
            f'{self.drone_namespace}/land',
            callback_group=self.callback_group
        )
        
        self.goto_action_client = ActionClient(
            self,
            GoToWaypoint,
            f'{self.drone_namespace}/goto_waypoint',
            callback_group=self.callback_group
        )
        
        self.execute_primitive_action_client = ActionClient(
            self,
            ExecutePrimitive,
            f'{self.drone_namespace}/execute_primitive',
            callback_group=self.callback_group
        )
        
        self.get_logger().info("FAL action clients created")
        
        # Initialize executors
        self.executors = {
            'waypoint': WaypointExecutor(self.config),
            'survey': SurveyExecutor(self.config),
            'search': SearchExecutor(self.config)
        }
        
        # Execution state
        self.current_primitives: List[PrimitiveCommand] = []
        self.primitive_index = 0
        
        # Thread safety
        self.execution_lock = Lock()
        
        # Home position (set on first GPS fix)
        self.home_position = None
        
        # Setup ROS communication
        self._setup_ros_communication()
        
        # Register state machine callbacks
        self._register_state_callbacks()
        
        # Register health monitor emergency callback
        self.health_monitor.register_emergency_callback(self._on_health_emergency)
        
        # Create main execution timer (20Hz)
        self.execution_timer = self.create_timer(
            0.05,  # 20Hz
            self._execution_loop,
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Task Execution Engine initialized successfully")
    
    def _load_default_config(self) -> Dict:
        """Load default configuration"""
        return {
            'drone': {
                'mass': 2.5,
                'max_speed': 15.0,
                'cruise_speed': 10.0,
                'hover_power': 250
            },
            'camera': {
                'sensor_width': 23.5,
                'sensor_height': 15.6,
                'focal_length': 16.0,
                'resolution_x': 6000,
                'resolution_y': 4000
            },
            'safety': {
                'min_battery_percentage': 0.25,
                'critical_battery_percentage': 0.20,
                'min_gps_satellites': 8,
                'max_gps_hdop': 2.0,
                'rtl_altitude': 50.0
            },
            'tasks': {
                'survey': {
                    'default_altitude': 50.0,
                    'default_overlap_forward': 0.75,
                    'default_overlap_side': 0.65
                },
                'search': {
                    'search_speed': 5.0,
                    'search_altitude': 30.0
                },
                'waypoint': {
                    'acceptance_radius': 2.0
                }
            }
        }
    
    def _setup_ros_communication(self):
        """Setup all ROS publishers and subscribers"""
        
        # Subscribers (inputs from Squadron Manager)
        self.mission_sub = self.create_subscription(
            String,
            '/squadron/mission_command',
            self._mission_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.pause_sub = self.create_subscription(
            Empty,
            '/squadron/pause',
            self._pause_callback,
            1,
            callback_group=self.callback_group
        )
        
        self.resume_sub = self.create_subscription(
            Empty,
            '/squadron/resume',
            self._resume_callback,
            1,
            callback_group=self.callback_group
        )
        
        self.abort_sub = self.create_subscription(
            Empty,
            '/squadron/abort',
            self._abort_callback,
            1,
            callback_group=self.callback_group
        )
        
        # Publishers (outputs to Squadron Manager)
        self.status_pub = self.create_publisher(
            String,
            '/tee/mission_status',
            10
        )
        
        self.get_logger().info("ROS communication setup complete")
    
    def _register_state_callbacks(self):
        """Register callbacks for state machine transitions"""
        
        # Entry callbacks
        self.state_machine.register_entry_callback(
            MissionState.VALIDATING,
            self._on_enter_validating
        )
        
        self.state_machine.register_entry_callback(
            MissionState.EXECUTING,
            self._on_enter_executing
        )
        
        self.state_machine.register_entry_callback(
            MissionState.EMERGENCY,
            self._on_enter_emergency
        )
        
        self.state_machine.register_entry_callback(
            MissionState.ABORTED,
            self._on_enter_aborted
        )
        
        self.state_machine.register_entry_callback(
            MissionState.COMPLETED,
            self._on_enter_completed
        )
        
        # Exit callbacks
        self.state_machine.register_exit_callback(
            MissionState.EXECUTING,
            self._on_exit_executing
        )
    
    # ============================================================================
    # ROS Callbacks
    # ============================================================================
    
    def _mission_callback(self, msg: String):
        """
        Handle incoming mission command from Squadron Manager.
        
        Args:
            msg: JSON string with mission command
        """
        try:
            mission_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse mission JSON: {e}")
            return
        
        self.get_logger().info(
            f"Received mission command: {mission_data.get('mission_id')} "
            f"({mission_data.get('task_type')})"
        )
        
        # Create prioritized task
        task = PrioritizedTask(
            priority=mission_data.get('priority', 50),
            task_id=mission_data.get('mission_id', f'mission_{self.get_clock().now().nanoseconds}'),
            task_type=mission_data.get('task_type', 'waypoint'),
            parameters=mission_data.get('parameters', {}),
            timeout=mission_data.get('timeout', 300.0)
        )
        
        # Enqueue task
        if self.task_queue.enqueue(task):
            self.get_logger().info(f"Task {task.task_id} enqueued successfully")
            
            # If idle, start validation
            if self.state_machine.is_state(MissionState.IDLE):
                self.state_machine.transition_to(
                    MissionState.VALIDATING,
                    reason=f"New mission received: {task.task_id}"
                )
        else:
            self.get_logger().error(f"Failed to enqueue task {task.task_id}")
    
    def _pause_callback(self, msg: Empty):
        """Handle pause command"""
        self.get_logger().info("Pause command received")
        
        if self.state_machine.is_state(MissionState.EXECUTING):
            self.state_machine.transition_to(
                MissionState.PAUSED,
                reason="User requested pause"
            )
        else:
            self.get_logger().warn(
                f"Cannot pause from state: {self.state_machine.current_state.value}"
            )
    
    def _resume_callback(self, msg: Empty):
        """Handle resume command"""
        self.get_logger().info("Resume command received")
        
        if self.state_machine.is_state(MissionState.PAUSED):
            self.state_machine.transition_to(
                MissionState.EXECUTING,
                reason="User requested resume"
            )
        else:
            self.get_logger().warn(
                f"Cannot resume from state: {self.state_machine.current_state.value}"
            )
    
    def _abort_callback(self, msg: Empty):
        """Handle abort command"""
        self.get_logger().warn("Abort command received")
        
        # Clear task queue
        self.task_queue.clear_queue()
        
        # Transition to aborted state
        if not self.state_machine.is_state(MissionState.IDLE):
            self.state_machine.transition_to(
                MissionState.ABORTED,
                reason="User requested abort"
            )
    
    # ============================================================================
    # State Machine Callbacks
    # ============================================================================
    
    def _on_enter_validating(self):
        """Called when entering VALIDATING state"""
        self.get_logger().info("Entering VALIDATING state")
    
    def _on_enter_executing(self):
        """Called when entering EXECUTING state"""
        self.get_logger().info("Entering EXECUTING state")
    
    def _on_enter_emergency(self):
        """Called when entering EMERGENCY state"""
        self.get_logger().error("Entering EMERGENCY state")
    
    def _on_enter_aborted(self):
        """Called when entering ABORTED state"""
        self.get_logger().warn("Entering ABORTED state")
        
        # Publish final status BEFORE clearing active_task
        if self.task_queue.active_task:
            abort_status = {
                'state': 'aborted',
                'mission_id': self.task_queue.active_task.task_id,
                'progress': self.progress_monitor.get_completion_percentage(),
                'health': self.health_monitor.get_overall_health(),
                'error': 'Mission aborted',
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            }
            msg = String()
            msg.data = json.dumps(abort_status)
            self.status_pub.publish(msg)
            
            self.get_logger().warn(
                f"Published abort status for mission: "
                f"{self.task_queue.active_task.task_id}"
            )
    
    def _on_enter_completed(self):
        """Called when entering COMPLETED state"""
        self.get_logger().info("Entering COMPLETED state")
        
        # Publish final status BEFORE clearing active_task
        # This ensures Squadron Manager receives the completion notification
        if self.task_queue.active_task:
            completion_status = {
                'state': 'completed',
                'mission_id': self.task_queue.active_task.task_id,
                'progress': 100.0,
                'health': self.health_monitor.get_overall_health(),
                'timestamp': self.get_clock().now().nanoseconds / 1e9
            }
            msg = String()
            msg.data = json.dumps(completion_status)
            self.status_pub.publish(msg)
            
            self.get_logger().info(
                f"Published completion status for mission: "
                f"{self.task_queue.active_task.task_id}"
            )
        
        # Mark task as completed (this clears active_task)
        if self.task_queue.active_task:
            self.task_queue.mark_completed(self.task_queue.active_task.task_id)
        
        # Complete progress tracking
        self.progress_monitor.complete_mission()
    
    def _on_exit_executing(self):
        """Called when exiting EXECUTING state"""
        self.get_logger().info("Exiting EXECUTING state")
    
    # ============================================================================
    # Main Execution Loop
    # ============================================================================
    
    def _execution_loop(self):
        """Main execution loop (called at 20Hz)"""
        try:
            # Process current state
            self._process_current_state()
            
            # Publish status
            self._publish_mission_status()
            
            # Check overall health
            overall_health = self.health_monitor.get_overall_health()
            if overall_health == 'critical' and not self.state_machine.is_state(MissionState.EMERGENCY):
                self.get_logger().error("Critical health detected - triggering emergency")
                self._trigger_emergency("Critical system health")
        
        except Exception as e:
            self.get_logger().error(f"Error in execution loop: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _process_current_state(self):
        """Process actions based on current state"""
        
        current_state = self.state_machine.current_state
        
        if current_state == MissionState.IDLE:
            self._handle_idle_state()
        
        elif current_state == MissionState.VALIDATING:
            self._handle_validating_state()
        
        elif current_state == MissionState.EXECUTING:
            self._handle_executing_state()
        
        elif current_state == MissionState.PAUSED:
            self._handle_paused_state()
        
        elif current_state == MissionState.EMERGENCY:
            self._handle_emergency_state()
        
        elif current_state == MissionState.COMPLETED:
            self._handle_completed_state()
        
        elif current_state == MissionState.ABORTED:
            self._handle_aborted_state()
    
    def _handle_idle_state(self):
        """Handle IDLE state - waiting for missions"""
        
        # Check if there are queued tasks
        if self.task_queue.has_tasks():
            self.state_machine.transition_to(
                MissionState.VALIDATING,
                reason="Tasks available in queue"
            )
    
    def _handle_validating_state(self):
        """Handle VALIDATING state - validate next task"""
        
        # Get next task from queue
        task = self.task_queue.get_next_task()
        
        if not task:
            # No tasks available, abort validation
            self.state_machine.transition_to(
                MissionState.ABORTED,
                reason="No tasks available"
            )
            return
        
        self.get_logger().info(f"Validating task {task.task_id}")
        
        # Get current monitor statuses
        battery_status = self.battery_monitor.get_current_status()
        gps_status = self.gps_monitor.get_current_status()
        
        # Validate task
        validation_result = self.validator.validate_task(
            task,
            battery_status=battery_status,
            gps_status=gps_status,
            connected=True  # TODO: Get actual connection status
        )
        
        if not validation_result.valid:
            # Validation failed
            self.get_logger().error(f"Task {task.task_id} validation failed")
            for failure in validation_result.failures:
                self.get_logger().error(f"  - {failure.message}")
            
            # Mark task as failed
            self.task_queue.mark_failed(
                task.task_id,
                reason=f"Validation failed: {validation_result.failures[0].message}"
            )
            
            # Abort mission
            self.state_machine.transition_to(
                MissionState.ABORTED,
                reason="Task validation failed"
            )
            return
        
        # Log warnings
        if validation_result.warnings:
            for warning in validation_result.warnings:
                self.get_logger().warn(f"Validation warning: {warning.message}")
        
        # Validation passed - generate full primitive sequence
        self.get_logger().info(f"Task {task.task_id} validation passed. Generating primitive sequence.")
        
        try:
            executor = self.executors.get(task.task_type)
            if not executor:
                raise ValueError(f"No executor for task type: {task.task_type}")
            
            # 1. Generate the mission-specific primitives (e.g., Goto commands)
            mission_primitives = executor.execute(task.parameters)
            
            # 2. Create the full sequence: Arm -> Takeoff -> Mission -> Land
            self.current_primitives = []
            
            # ARM
            arm_cmd = PrimitiveCommand()
            arm_cmd.primitive_type = "arm"
            arm_cmd.command_id = f"{task.task_id}_arm"
            self.current_primitives.append(arm_cmd)
            
            # TAKEOFF (Use a default altitude from config)
            takeoff_alt = self.config.get('safety', {}).get('rtl_altitude', 10.0)
            takeoff_cmd = PrimitiveCommand()
            takeoff_cmd.primitive_type = "takeoff"
            takeoff_cmd.command_id = f"{task.task_id}_takeoff"
            takeoff_cmd.target_position.z = float(takeoff_alt)
            self.current_primitives.append(takeoff_cmd)
            
            # MISSION
            self.current_primitives.extend(mission_primitives)
            
            # LAND
            land_cmd = PrimitiveCommand()
            land_cmd.primitive_type = "land"
            land_cmd.command_id = f"{task.task_id}_land"
            self.current_primitives.append(land_cmd)
            
            self.primitive_index = 0
            
            self.get_logger().info(f"Generated {len(self.current_primitives)} total primitives (Arm, Takeoff, {len(mission_primitives)} mission, Land)")
            
            # Start progress tracking
            self.progress_monitor.start_mission(
                task_id=task.task_id,
                task_type=task.task_type,
                task_parameters=task.parameters,
                total_primitives=len(self.current_primitives)
            )
            
            # Transition to executing
            self.state_machine.transition_to(
                MissionState.EXECUTING,
                reason=f"Validation passed, starting execution"
            )
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate primitives: {e}")
            self.task_queue.mark_failed(task.task_id, reason=str(e))
            self.state_machine.transition_to(
                MissionState.ABORTED,
                reason="Primitive generation failed"
            )
    
    def _handle_executing_state(self):
        """Handle EXECUTING state - execute primitives using action clients"""
        
        # This lock prevents the 20Hz timer from trying to run a new
        # primitive while the last one is still blocking/executing.
        if not self.execution_lock.acquire(blocking=False):
            # Last primitive is still running, wait for it to finish.
            return
        
        try:
            # Check if all primitives completed
            if self.primitive_index >= len(self.current_primitives):
                self.get_logger().info("All primitives executed successfully")
                self.state_machine.transition_to(
                    MissionState.COMPLETED,
                    reason="All primitives completed"
                )
                return
            
            # Get next primitive
            primitive = self.current_primitives[self.primitive_index]
            
            self.get_logger().info(
                f"Executing primitive {self.primitive_index + 1}/{len(self.current_primitives)}: "
                f"{primitive.primitive_type.upper()}"
            )
            
            # Execute the primitive and BLOCK until it's done
            # This call is safe because it's waiting for a *different node* (FAL)
            success = self._execute_primitive_blocking(primitive)
            
            if success:
                self.get_logger().info(f"Primitive {primitive.primitive_type.upper()} completed successfully.")
                self.primitive_index += 1
                self.progress_monitor.update_primitive_completed(self.primitive_index - 1)
            else:
                self.get_logger().error(f"Primitive {primitive.primitive_type.upper()} failed.")
                self._handle_primitive_failure(f"Primitive {primitive.primitive_type} failed")
        
        except Exception as e:
            self.get_logger().error(f"Exception in execution state: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            self._handle_primitive_failure(str(e))
        
        finally:
            self.execution_lock.release()
    
    # ============================================================================
    # Primitive Execution (Action-Based)
    # ============================================================================
    
    def _execute_primitive_blocking(self, primitive: PrimitiveCommand) -> bool:
        """
        Selects and runs the correct blocking executor for a primitive.
        Returns True on success, False on failure.
        """
        
        primitive_type = primitive.primitive_type.lower()
        
        if primitive_type == "arm":
            return self._execute_arm(primitive)
        elif primitive_type == "takeoff":
            return self._execute_takeoff(primitive)
        elif primitive_type == "goto":
            return self._execute_goto(primitive)
        elif primitive_type == "land":
            return self._execute_land(primitive)
        else:
            self.get_logger().error(f"Unknown primitive type: {primitive_type}")
            return False
    
    def _execute_arm(self, primitive: PrimitiveCommand) -> bool:
        """Executes the ARM service and blocks until complete."""
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Arm service not available")
            return False
        
        req = ArmDisarm.Request()
        req.arm = True
        req.force = False
        
        future = self.arm_client.call_async(req)
        
        # Poll the future without blocking the node
        timeout = 15.0
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not future.done():
            time.sleep(0.05)  # 50ms polling
            elapsed = self.get_clock().now().nanoseconds / 1e9 - start_time
            if elapsed > timeout:
                self.get_logger().error("Arm service call timed out")
                return False
        
        if future.result() is None:
            self.get_logger().error("Arm service returned None")
            return False
        
        return future.result().success
    
    def _execute_takeoff(self, primitive: PrimitiveCommand) -> bool:
        """Executes the TAKEOFF action and blocks until complete."""
        if not self.takeoff_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Takeoff action server not available")
            return False
        
        goal = Takeoff.Goal()
        goal.target_altitude = float(primitive.target_position.z)
        
        # Send goal
        goal_future = self.takeoff_action_client.send_goal_async(goal)
        
        # Poll without blocking
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not goal_future.done():
            time.sleep(0.05)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 10.0:
                self.get_logger().error("Takeoff goal send timed out")
                return False
        
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Takeoff goal rejected")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not result_future.done():
            time.sleep(0.05)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 60.0:
                self.get_logger().error("Takeoff action timed out")
                return False
        
        return result_future.result().result.success
    
    def _execute_goto(self, primitive: PrimitiveCommand) -> bool:
        """Executes the GOTO action and blocks until complete."""
        if not self.goto_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Goto action server not available")
            return False
        
        goal = GoToWaypoint.Goal()
        goal.target_position = primitive.target_position
        goal.target_heading = -1.0  # -1 = don't change heading
        goal.max_speed = primitive.velocity if primitive.velocity > 0 else 2.0
        goal.acceptance_radius = primitive.acceptance_radius if primitive.acceptance_radius > 0 else 1.0
        
        # Send goal
        goal_future = self.goto_action_client.send_goal_async(goal)
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not goal_future.done():
            time.sleep(0.05)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 10.0:
                self.get_logger().error("Goto goal send timed out")
                return False
        
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goto goal rejected")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not result_future.done():
            time.sleep(0.05)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 120.0:
                self.get_logger().error("Goto action timed out")
                return False
        
        return result_future.result().result.success
    
    def _execute_land(self, primitive: PrimitiveCommand) -> bool:
        """Executes the LAND action and blocks until complete."""
        if not self.land_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Land action server not available")
            return False
        
        goal = Land.Goal()
        goal.descent_rate = 0.5  # Default descent rate of 0.5 m/s
        
        # Send goal
        goal_future = self.land_action_client.send_goal_async(goal)
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not goal_future.done():
            time.sleep(0.05)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 10.0:
                self.get_logger().error("Land goal send timed out")
                return False
        
        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Land goal rejected")
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        start_time = self.get_clock().now().nanoseconds / 1e9
        while not result_future.done():
            time.sleep(0.05)
            if (self.get_clock().now().nanoseconds / 1e9 - start_time) > 120.0:
                self.get_logger().error("Land action timed out")
                return False
        
        return result_future.result().result.success
    
    # ============================================================================
    # State Handlers (Continued)
    # ============================================================================
    
    def _handle_paused_state(self):
        """Handle PAUSED state - hold position"""
        # Nothing to do - just maintain state
        pass
    
    def _handle_emergency_state(self):
        """Handle EMERGENCY state - execute emergency procedures"""
        if not self.execution_lock.acquire(blocking=False):
            # Emergency landing is already in progress
            return
        
        try:
            self.get_logger().critical("EXECUTING EMERGENCY LAND")
            
            # Create an emergency land primitive
            land_primitive = PrimitiveCommand()
            land_primitive.primitive_type = "land"
            land_primitive.command_id = "emergency_land"
            
            # Execute it
            success = self._execute_land(land_primitive)
            
            if success:
                self.get_logger().info("Emergency land successful.")
                self.state_machine.transition_to(MissionState.ABORTED, reason="Emergency land complete")
            else:
                self.get_logger().critical("EMERGENCY LAND FAILED. DRONE IS IN AN UNSTABLE STATE.")
                # We are stuck in emergency state, which is appropriate.
        
        finally:
            self.execution_lock.release()
    
    def _handle_completed_state(self):
        """Handle COMPLETED state - clean up and return to idle"""
        
        # Clean up current execution
        self.current_primitives = []
        self.primitive_index = 0
        
        # Check for more tasks
        if self.task_queue.has_tasks():
            self.get_logger().info("More tasks in queue, transitioning to validating")
            self.state_machine.transition_to(
                MissionState.VALIDATING,
                reason="Additional tasks queued"
            )
        else:
            self.get_logger().info("No more tasks, transitioning to idle")
            self.state_machine.transition_to(
                MissionState.IDLE,
                reason="All tasks completed"
            )
    
    def _handle_aborted_state(self):
        """Handle ABORTED state - clean up after abort"""
        
        # Clean up
        self.current_primitives = []
        self.primitive_index = 0
        
        # Return to idle
        self.state_machine.transition_to(
            MissionState.IDLE,
            reason="Abort cleanup complete"
        )
    
    # ============================================================================
    # Emergency Handling
    # ============================================================================
    
    def _on_health_emergency(self, reason: str, status):
        """
        Called by health monitor on critical health state.
        
        Args:
            reason: Emergency reason
            status: Health status object
        """
        self.get_logger().error(f"Health emergency: {reason}")
        self._trigger_emergency(reason)
    
    def _trigger_emergency(self, reason: str):
        """
        Trigger emergency procedures.
        
        Args:
            reason: Reason for emergency
        """
        self.get_logger().error(f"EMERGENCY TRIGGERED: {reason}")
        
        # Create emergency RTL task
        emergency_rtl = PrioritizedTask(
            priority=TaskQueue.PRIORITY_EMERGENCY,
            task_id=f"emergency_rtl_{self.get_clock().now().nanoseconds}",
            task_type='rtl',
            parameters={
                'home_position': self.home_position if self.home_position else [0.0, 0.0, 0.0]
            }
        )
        
        # Inject emergency task
        self.task_queue.inject_emergency_task(emergency_rtl)
        
        # Transition to emergency state
        self.state_machine.transition_to(
            MissionState.EMERGENCY,
            reason=reason
        )
    
    def _handle_primitive_failure(self, error_message: str):
        """
        Handle primitive execution failure.
        
        Args:
            error_message: Error description
        """
        self.get_logger().error(f"Primitive failure: {error_message}")
        
        # Mark current task as failed
        if self.task_queue.active_task:
            self.task_queue.mark_failed(
                self.task_queue.active_task.task_id,
                reason=error_message
            )
        
        # Transition to aborted
        self.state_machine.transition_to(
            MissionState.ABORTED,
            reason=f"Primitive failure: {error_message}"
        )
    
    # ============================================================================
    # Status Publishing
    # ============================================================================
    
    def _publish_mission_status(self):
        """Publish current mission status"""
        
        status = {
            'state': self.state_machine.current_state.value,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        # Add task information if available
        if self.task_queue.active_task:
            status['mission_id'] = self.task_queue.active_task.task_id
            
            # Add progress
            progress = self.progress_monitor.get_completion_percentage()
            status['progress'] = progress
        
        # Add health summary
        status['health'] = self.health_monitor.get_overall_health()
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        # Create TEE node
        tee = TaskExecutionEngineNode()
        
        # Use multi-threaded executor
        executor = MultiThreadedExecutor()
        executor.add_node(tee)
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            tee.destroy_node()
        
    except KeyboardInterrupt:
        tee.get_logger().info("TEE node interrupted")
    except Exception as e:
        print(f"TEE node failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
