#!/usr/bin/env python3
"""
Squadron Manager Node - Coordinates multi-drone operations
"""

import json
import copy
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Squadron components
from squadron_manager.drone_registry import DroneRegistry, DroneState, DroneCapabilities
from squadron_manager.task_allocator import TaskAllocator, AllocationStrategy, TaskRequirements
from squadron_manager.formation_controller import FormationController, FormationType, FormationParameters
from squadron_manager.sync_barrier import BarrierManager, SyncBarrier

# ROS messages
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState, NavSatFix
from mavros_msgs.msg import State


class SquadronManagerNode(Node):
    """
    Squadron Manager - Central coordination for multi-drone operations.
    
    Responsibilities:
    - Manage drone registry and status
    - Allocate tasks to available drones
    - Coordinate formation flying
    - Monitor squadron health
    - Send missions to individual TEE nodes
    """
    
    def __init__(self):
        """Initialize Squadron Manager"""
        super().__init__('squadron_manager')
        
        self.get_logger().info("Initializing Squadron Manager...")
        
        # Parameters
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('allocation_strategy', 'nearest')
        self.declare_parameter('enable_formations', True)
        self.declare_parameter('enable_task_reallocation', True)
        
        self.num_drones = self.get_parameter('num_drones').value
        strategy_str = self.get_parameter('allocation_strategy').value
        self.enable_formations = self.get_parameter('enable_formations').value
        
        # Initialize components
        self.drone_registry = DroneRegistry(self.get_logger())
        
        # Map strategy string to enum
        strategy_map = {
            'greedy': AllocationStrategy.GREEDY,
            'nearest': AllocationStrategy.NEAREST,
            'load_balanced': AllocationStrategy.LOAD_BALANCED,
            'capability_based': AllocationStrategy.CAPABILITY_BASED,
            'optimal': AllocationStrategy.OPTIMAL
        }
        strategy = strategy_map.get(strategy_str, AllocationStrategy.NEAREST)
        self.task_allocator = TaskAllocator(self.get_logger(), strategy)
        
        self.formation_controller = FormationController(self.get_logger())
        
        # Initialize sync barrier manager for coordinated operations
        self.barrier_manager = BarrierManager(self.get_logger())
        
        # Store active missions for potential re-allocation
        # Key: mission_id, Value: full mission dict
        self.active_missions: Dict[str, Dict] = {}
        
        # Track mission assignments: drone_id -> mission_id
        self.drone_mission_map: Dict[str, str] = {}
        
        # QoS profiles
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Register drones and create subscribers
        self._register_drones()
        self._setup_subscribers()
        self._setup_publishers()
        
        # Create timer for health monitoring
        self.create_timer(1.0, self._monitor_squadron_health)
        
        self.get_logger().info(
            f"Squadron Manager initialized with {self.num_drones} drones "
            f"(strategy: {strategy_str}, formations: {self.enable_formations})"
        )
    
    def _register_drones(self):
        """Register all drones in the squadron"""
        for i in range(self.num_drones):
            drone_id = f"drone_{i}"
            namespace = f"/drone_{i}"
            
            # Create default capabilities
            capabilities = DroneCapabilities(
                max_speed=5.0,
                max_altitude=100.0,
                flight_time=20.0,
                payload_capacity=0.5,
                has_camera=True,
                has_lidar=False,
                has_gripper=False
            )
            
            self.drone_registry.register_drone(drone_id, namespace, capabilities)
    
    def _setup_subscribers(self):
        """Setup subscribers for all drones"""
        self.state_subs = {}
        self.position_subs = {}
        self.battery_subs = {}
        self.gps_subs = {}
        self.mission_status_subs = {}
        
        for i in range(self.num_drones):
            drone_id = f"drone_{i}"
            namespace = f"/drone_{i}"
            
            # MAVROS state
            self.state_subs[drone_id] = self.create_subscription(
                State,
                f'{namespace}/mavros/state',
                lambda msg, did=drone_id: self._state_callback(msg, did),
                self.mavros_qos
            )
            
            # Local position
            self.position_subs[drone_id] = self.create_subscription(
                PoseStamped,
                f'{namespace}/mavros/local_position/pose',
                lambda msg, did=drone_id: self._position_callback(msg, did),
                self.mavros_qos
            )
            
            # Battery state
            self.battery_subs[drone_id] = self.create_subscription(
                BatteryState,
                f'{namespace}/mavros/battery',
                lambda msg, did=drone_id: self._battery_callback(msg, did),
                self.mavros_qos
            )
            
            # GPS status
            self.gps_subs[drone_id] = self.create_subscription(
                NavSatFix,
                f'{namespace}/mavros/global_position/global',
                lambda msg, did=drone_id: self._gps_callback(msg, did),
                self.mavros_qos
            )
            
            # Mission status from TEE (if running)
            self.mission_status_subs[drone_id] = self.create_subscription(
                String,
                f'{namespace}/tee/mission_status',
                lambda msg, did=drone_id: self._mission_status_callback(msg, did),
                self.reliable_qos
            )
        
        # Subscribe to high-level mission commands
        self.mission_command_sub = self.create_subscription(
            String,
            '/squadron/mission_command',
            self._mission_command_callback,
            self.reliable_qos
        )
        
        # Subscribe to formation commands
        if self.enable_formations:
            self.formation_command_sub = self.create_subscription(
                String,
                '/squadron/formation_command',
                self._formation_command_callback,
                self.reliable_qos
            )
    
    def _setup_publishers(self):
        """Setup publishers for squadron status and drone commands"""
        # Squadron status publisher
        self.squadron_status_pub = self.create_publisher(
            String,
            '/squadron/status',
            self.reliable_qos
        )
        
        # Individual drone mission publishers (to TEE)
        self.drone_mission_pubs = {}
        for i in range(self.num_drones):
            drone_id = f"drone_{i}"
            namespace = f"/drone_{i}"
            
            # Publish to each drone's TEE mission command topic
            self.drone_mission_pubs[drone_id] = self.create_publisher(
                String,
                f'{namespace}/tee/mission_command',
                self.reliable_qos
            )
        
        # Timer to publish squadron status
        self.create_timer(2.0, self._publish_squadron_status)
    
    def _state_callback(self, msg: State, drone_id: str):
        """Handle MAVROS state updates"""
        self.get_logger().debug(
            f"State callback for {drone_id}: connected={msg.connected}, "
            f"armed={msg.armed}, mode={msg.mode}"
        )
        self.drone_registry.update_drone_connection(
            drone_id=drone_id,
            connected=msg.connected,
            armed=msg.armed,
            mode=msg.mode
        )
        self.get_logger().debug(f"Registry updated for {drone_id}")
    
    def _position_callback(self, msg: PoseStamped, drone_id: str):
        """Handle position updates"""
        self.drone_registry.update_drone_position(
            drone_id=drone_id,
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z
        )
    
    def _battery_callback(self, msg: BatteryState, drone_id: str):
        """Handle battery updates"""
        # Calculate percentage and remaining time
        percentage = msg.percentage * 100.0 if msg.percentage > 0 else 0.0
        
        self.get_logger().debug(
            f"Battery callback for {drone_id}: {percentage:.1f}% "
            f"(voltage={msg.voltage:.2f}V)"
        )
        
        # Estimate remaining time (simplified)
        # Assuming 20 minutes flight time at 100%
        remaining_time = 20.0 * (percentage / 100.0)
        
        self.drone_registry.update_drone_battery(
            drone_id=drone_id,
            voltage=msg.voltage,
            percentage=percentage,
            remaining_time=remaining_time
        )
    
    def _gps_callback(self, msg: NavSatFix, drone_id: str):
        """Handle GPS updates"""
        # NavSatFix status: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
        # For squadron manager, we consider status >= 0 as valid (fix_type >= 2 in our system)
        # Map: status -1 -> fix_type 0, status 0 -> fix_type 2, status 1+ -> fix_type 3+
        fix_type = max(0, msg.status.status + 2)
        
        # Extract number of satellites (if available in covariance type)
        # This is simplified - actual implementation may need more info
        satellites = 10  # SITL default (assuming good fix)
        
        self.get_logger().debug(
            f"GPS callback for {drone_id}: fix_type={fix_type}, "
            f"lat={msg.latitude:.6f}, lon={msg.longitude:.6f}"
        )
        
        self.drone_registry.update_drone_gps(
            drone_id=drone_id,
            fix_type=fix_type,
            satellites=satellites
        )
    
    def _mission_status_callback(self, msg: String, drone_id: str):
        """Handle mission status updates from TEE"""
        try:
            status = json.loads(msg.data)
            mission_id = status.get('mission_id', '')
            state = status.get('state', '')
            
            self.get_logger().debug(
                f"Mission status from {drone_id}: mission={mission_id}, "
                f"state={state}, progress={status.get('progress', 0):.1f}%"
            )
            
            # Handle sync barrier arrivals
            if 'barrier_arrived' in status:
                barrier_id = status['barrier_arrived']
                self.handle_drone_barrier_arrival(drone_id, barrier_id)
            
            # Update task progress
            if 'progress' in status:
                self.drone_registry.update_task_progress(
                    drone_id=drone_id,
                    progress=status['progress']
                )
            
            # Check if mission completed
            if state == 'completed':
                self.get_logger().info(
                    f"✓ Mission {mission_id} completed by {drone_id} - "
                    f"clearing task assignment"
                )
                self.drone_registry.complete_task(drone_id)
                self.task_allocator.deallocate_task(mission_id)
                
                # Clean up stored mission
                if mission_id in self.active_missions:
                    del self.active_missions[mission_id]
                if drone_id in self.drone_mission_map:
                    del self.drone_mission_map[drone_id]
            
            # Check if mission failed/aborted
            elif state in ['aborted', 'failed']:
                self.get_logger().error(
                    f"✗ Mission {mission_id} failed on {drone_id} "
                    f"(reason: {status.get('error', 'unknown')})"
                )
                
                # Handle drone removal from barriers
                self._handle_drone_failure_barriers(drone_id)
                
                # Handle mission failure with potential re-allocation
                self._handle_mission_failure(drone_id, mission_id, status)
        
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid mission status from {drone_id}")
    
    def _handle_drone_failure_barriers(self, drone_id: str):
        """Handle a drone failure by removing it from all active barriers"""
        barriers_affected = []
        
        for barrier_id, barrier in self.barrier_manager.barriers.items():
            if drone_id in barrier.expected_drones:
                barrier.expected_drones.discard(drone_id)
                barrier.expected_count = len(barrier.expected_drones)
                barriers_affected.append(barrier_id)
        
        if barriers_affected:
            self.get_logger().warning(
                f"Removed failed drone {drone_id} from barriers: {barriers_affected}"
            )
    
    def _handle_mission_failure(self, failed_drone_id: str, mission_id: str, status: Dict):
        """
        Handle mission failure with potential re-allocation.
        
        Args:
            failed_drone_id: ID of drone that failed the mission
            mission_id: ID of the failed mission
            status: Status message from TEE with failure details
        """
        # Mark drone task as complete (failed)
        self.drone_registry.complete_task(failed_drone_id)
        self.task_allocator.deallocate_task(mission_id)
        
        # Clear from drone-mission map
        if failed_drone_id in self.drone_mission_map:
            del self.drone_mission_map[failed_drone_id]
        
        # Check if we should attempt re-allocation
        enable_reallocation = self.get_parameter('enable_task_reallocation').value
        
        if not enable_reallocation:
            self.get_logger().warning(
                f"Mission re-allocation disabled. Mission {mission_id} lost."
            )
            # Clean up stored mission
            if mission_id in self.active_missions:
                del self.active_missions[mission_id]
            return
        
        # Check if we have the original mission stored
        if mission_id not in self.active_missions:
            self.get_logger().error(
                f"Cannot re-allocate mission {mission_id}: "
                f"Original mission data not found"
            )
            return
        
        original_mission = self.active_missions[mission_id]
        
        # Get mission progress to determine remaining work
        progress = status.get('progress', 0.0)
        remaining_waypoints = status.get('remaining_waypoints', None)
        
        # Get available drones (excluding the failed one)
        available_drones = [
            d for d in self.drone_registry.get_available_drones()
            if d.drone_id != failed_drone_id
        ]
        
        if not available_drones:
            self.get_logger().error(
                f"Cannot re-allocate mission {mission_id}: No available drones"
            )
            return
        
        self.get_logger().info(
            f"Attempting to re-allocate failed mission {mission_id} "
            f"(progress: {progress:.1f}%) to another drone "
            f"({len(available_drones)} available)"
        )
        
        # Create supplementary mission with remaining work
        self._reallocate_mission(
            original_mission,
            available_drones,
            progress,
            remaining_waypoints
        )
    
    def _reallocate_mission(
        self,
        original_mission: Dict,
        available_drones: List,
        progress: float,
        remaining_waypoints: Optional[List] = None
    ):
        """
        Re-allocate a mission to a new drone.
        
        Args:
            original_mission: The original mission dictionary
            available_drones: List of available drones
            progress: How much of the mission was completed (0-100)
            remaining_waypoints: Specific remaining waypoints if available
        """
        mission_id = original_mission.get('mission_id', 'unknown')
        
        # Create a new mission based on remaining work
        new_mission = copy.deepcopy(original_mission)
        new_mission['mission_id'] = f"{mission_id}_reallocated"
        new_mission['original_mission_id'] = mission_id
        new_mission['is_reallocation'] = True
        
        # Update waypoints if we have remaining waypoint info
        if remaining_waypoints is not None:
            if 'parameters' not in new_mission:
                new_mission['parameters'] = {}
            new_mission['parameters']['waypoints'] = remaining_waypoints
        elif progress > 0:
            # Estimate remaining waypoints based on progress
            if 'parameters' in new_mission and 'waypoints' in new_mission['parameters']:
                waypoints = new_mission['parameters']['waypoints']
                completed_count = int(len(waypoints) * (progress / 100.0))
                new_mission['parameters']['waypoints'] = waypoints[completed_count:]
        
        # Create requirements for allocation
        requirements = TaskRequirements()
        
        if 'parameters' in new_mission and 'waypoints' in new_mission['parameters']:
            waypoints = new_mission['parameters']['waypoints']
            if waypoints:
                first_wp = waypoints[0]
                requirements.target_location = (
                    first_wp.get('x', 0.0),
                    first_wp.get('y', 0.0),
                    first_wp.get('z', 50.0)
                )
        
        # Allocate to best available drone
        selected_drone_id = self.task_allocator.allocate_task(
            task_id=new_mission['mission_id'],
            available_drones=available_drones,
            requirements=requirements
        )
        
        if not selected_drone_id:
            self.get_logger().error(
                f"Failed to find suitable drone for re-allocated mission"
            )
            return
        
        # Assign and send mission
        self.drone_registry.assign_task(selected_drone_id, new_mission['mission_id'])
        self.drone_mission_map[selected_drone_id] = new_mission['mission_id']
        self.active_missions[new_mission['mission_id']] = new_mission
        
        # Remove original mission
        if mission_id in self.active_missions:
            del self.active_missions[mission_id]
        
        self._send_mission_to_drone(selected_drone_id, new_mission)
        
        self.get_logger().info(
            f"✓ Re-allocated mission {mission_id} → {new_mission['mission_id']} "
            f"to {selected_drone_id}"
        )
    
    def _mission_command_callback(self, msg: String):
        """
        Handle high-level mission commands.
        Parse mission and allocate to appropriate drone(s).
        """
        try:
            mission = json.loads(msg.data)
            
            mission_id = mission.get('mission_id', 'unknown')
            task_type = mission.get('task_type', 'waypoint')
            parameters = mission.get('parameters', {})
            
            self.get_logger().info(
                f"Received mission: {mission_id} (type: {task_type})"
            )
            
            # Determine if this is single-drone or multi-drone mission
            multi_drone = mission.get('multi_drone', False)
            
            if multi_drone:
                self._handle_multi_drone_mission(mission)
            else:
                self._handle_single_drone_mission(mission)
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid mission JSON: {e}")
    
    def _handle_single_drone_mission(self, mission: Dict):
        """Allocate and execute a single-drone mission"""
        mission_id = mission.get('mission_id', 'unknown')
        parameters = mission.get('parameters', {})
        
        # Get available drones
        available_drones = self.drone_registry.get_available_drones()
        
        if not available_drones:
            self.get_logger().error(f"No available drones for mission {mission_id}")
            
            # Debug: Show why drones are not available
            all_drones = self.drone_registry.get_all_drones()
            for drone in all_drones:
                self.get_logger().warn(
                    f"Drone {drone.drone_id}: state={drone.state.name}, "
                    f"connected={drone.connected}, battery={drone.battery_percentage:.1f}%, "
                    f"gps_fix={drone.gps_fix_type}, task={drone.current_task_id}"
                )
            return
        
        # Create task requirements
        requirements = TaskRequirements()
        
        # Extract target location if available
        if 'waypoints' in parameters and len(parameters['waypoints']) > 0:
            wp = parameters['waypoints'][0]
            requirements.target_location = (
                wp.get('x', 0.0),
                wp.get('y', 0.0),
                wp.get('z', 50.0)
            )
        
        # Allocate task to best drone
        selected_drone_id = self.task_allocator.allocate_task(
            task_id=mission_id,
            available_drones=available_drones,
            requirements=requirements
        )
        
        if not selected_drone_id:
            self.get_logger().error(f"Failed to allocate mission {mission_id}")
            return
        
        # Assign task in registry
        self.drone_registry.assign_task(selected_drone_id, mission_id)
        
        # Store mission for potential re-allocation
        self.active_missions[mission_id] = copy.deepcopy(mission)
        self.drone_mission_map[selected_drone_id] = mission_id
        
        # Send mission to selected drone's TEE
        self._send_mission_to_drone(selected_drone_id, mission)
    
    def _handle_multi_drone_mission(self, mission: Dict):
        """Coordinate a multi-drone mission with sync barriers"""
        mission_id = mission.get('mission_id', 'unknown')
        formation_type = mission.get('formation_type', 'line')
        
        self.get_logger().info(
            f"Executing multi-drone mission {mission_id} with {formation_type} formation"
        )
        
        # Get available drones
        available_drones = self.drone_registry.get_available_drones()
        
        if len(available_drones) < 2:
            self.get_logger().error("Need at least 2 drones for multi-drone mission")
            return
        
        drone_ids = [d.drone_id for d in available_drones]
        
        # Create synchronization barriers for this mission
        # Barrier 1: All drones must be ready before takeoff
        takeoff_barrier = self.barrier_manager.create_barrier(
            name=f"{mission_id}_takeoff",
            num_drones=len(drone_ids),
            timeout=30.0,
            on_complete=lambda b: self._on_barrier_complete(b, 'takeoff'),
            on_timeout=lambda b: self._on_barrier_timeout(b, 'takeoff')
        )
        
        # Barrier 2: All drones reach formation altitude before proceeding
        altitude_barrier = self.barrier_manager.create_barrier(
            name=f"{mission_id}_altitude",
            num_drones=len(drone_ids),
            timeout=60.0,
            on_complete=lambda b: self._on_barrier_complete(b, 'altitude'),
            on_timeout=lambda b: self._on_barrier_timeout(b, 'altitude')
        )
        
        # Barrier 3: Formation established before mission execution
        formation_barrier = self.barrier_manager.create_barrier(
            name=f"{mission_id}_formation",
            num_drones=len(drone_ids),
            timeout=45.0,
            on_complete=lambda b: self._on_barrier_complete(b, 'formation'),
            on_timeout=lambda b: self._on_barrier_timeout(b, 'formation')
        )
        
        self.get_logger().info(
            f"Created sync barriers for mission {mission_id}: "
            f"takeoff, altitude, formation"
        )
        
        # Create formation parameters
        formation_params = FormationParameters(
            formation_type=FormationType(formation_type),
            spacing=mission.get('spacing', 10.0),
            altitude=mission.get('altitude', 50.0)
        )
        
        self.formation_controller.create_formation(available_drones, formation_params)
        
        # Generate and send individual missions for each drone
        base_waypoints = mission.get('parameters', {}).get('waypoints', [])
        formation_positions = self.formation_controller.get_formation_positions(
            available_drones,
            formation_params
        )
        
        for i, drone in enumerate(available_drones):
            drone_id = drone.drone_id
            
            # Create individual mission with sync points
            individual_mission = {
                'mission_id': f"{mission_id}_{drone_id}",
                'parent_mission_id': mission_id,
                'task_type': mission.get('task_type', 'formation_flight'),
                'sync_barriers': {
                    'takeoff': f"{mission_id}_takeoff",
                    'altitude': f"{mission_id}_altitude",
                    'formation': f"{mission_id}_formation"
                },
                'formation_position': formation_positions.get(drone_id, i),
                'parameters': {
                    'waypoints': self._offset_waypoints(
                        base_waypoints,
                        formation_positions.get(drone_id, (i * 5.0, 0.0, 0.0))
                    ),
                    'altitude': mission.get('altitude', 50.0),
                    'speed': mission.get('speed', 3.0)
                }
            }
            
            # Assign and track
            self.drone_registry.assign_task(drone_id, individual_mission['mission_id'])
            self.active_missions[individual_mission['mission_id']] = individual_mission
            self.drone_mission_map[drone_id] = individual_mission['mission_id']
            
            # Send to drone
            self._send_mission_to_drone(drone_id, individual_mission)
        
        self.get_logger().info(
            f"Multi-drone mission {mission_id} distributed to {len(available_drones)} drones "
            f"with {len(base_waypoints)} waypoints per drone"
        )
    
    def _offset_waypoints(
        self,
        waypoints: List[Dict],
        offset: tuple
    ) -> List[Dict]:
        """Offset waypoints by formation position"""
        offset_waypoints = []
        for wp in waypoints:
            new_wp = copy.deepcopy(wp)
            new_wp['x'] = wp.get('x', 0.0) + offset[0]
            new_wp['y'] = wp.get('y', 0.0) + offset[1]
            # Z offset is usually handled differently (keep same altitude)
            offset_waypoints.append(new_wp)
        return offset_waypoints
    
    def _on_barrier_complete(self, barrier: SyncBarrier, barrier_type: str):
        """Handle barrier completion"""
        self.get_logger().info(
            f"✓ Barrier '{barrier.barrier_id}' ({barrier_type}) completed - "
            f"all {len(barrier.participants)} drones synchronized"
        )
        
        # Publish barrier completion event
        msg = String()
        msg.data = json.dumps({
            'event': 'barrier_complete',
            'barrier_id': barrier.barrier_id,
            'barrier_type': barrier_type,
            'participants': list(barrier.participants)
        })
        self.squadron_status_pub.publish(msg)
    
    def _on_barrier_timeout(self, barrier: SyncBarrier, barrier_type: str):
        """Handle barrier timeout"""
        missing = barrier.get_missing_participants()
        self.get_logger().error(
            f"✗ Barrier '{barrier.barrier_id}' ({barrier_type}) TIMEOUT - "
            f"missing drones: {missing}"
        )
        
        # Consider missing drones as potentially failed
        for drone_id in missing:
            self.get_logger().warning(
                f"Drone {drone_id} failed to reach {barrier_type} barrier - "
                f"may need re-allocation"
            )
        
        # Publish barrier timeout event
        msg = String()
        msg.data = json.dumps({
            'event': 'barrier_timeout',
            'barrier_id': barrier.barrier_id,
            'barrier_type': barrier_type,
            'missing_participants': missing,
            'arrived_participants': list(barrier.arrived)
        })
        self.squadron_status_pub.publish(msg)
    
    def handle_drone_barrier_arrival(self, drone_id: str, barrier_id: str):
        """
        Handle a drone arriving at a barrier.
        Called when TEE reports reaching a sync point.
        """
        barrier = self.barrier_manager.get_barrier(barrier_id)
        if barrier:
            barrier.arrive(drone_id)
            self.get_logger().debug(
                f"Drone {drone_id} arrived at barrier {barrier_id} "
                f"({len(barrier.arrived)}/{len(barrier.participants)})"
            )
    
    def _formation_command_callback(self, msg: String):
        """Handle formation control commands"""
        try:
            command = json.loads(msg.data)
            
            cmd_type = command.get('type', 'create')
            
            if cmd_type == 'create':
                self._create_formation(command)
            elif cmd_type == 'break':
                self._break_formation()
            elif cmd_type == 'update_center':
                center = command.get('center', [0.0, 0.0, 50.0])
                self.formation_controller.update_formation_center(tuple(center))
            else:
                self.get_logger().warning(f"Unknown formation command: {cmd_type}")
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid formation command JSON: {e}")
    
    def _create_formation(self, command: Dict):
        """Create a formation from command"""
        available_drones = self.drone_registry.get_available_drones()
        
        formation_params = FormationParameters(
            formation_type=FormationType(command.get('formation_type', 'line')),
            spacing=command.get('spacing', 10.0),
            altitude=command.get('altitude', 50.0),
            heading=command.get('heading', 0.0),
            center=tuple(command.get('center', [0.0, 0.0, 50.0]))
        )
        
        success = self.formation_controller.create_formation(
            available_drones,
            formation_params
        )
        
        if success:
            self.get_logger().info("Formation created successfully")
        else:
            self.get_logger().error("Failed to create formation")
    
    def _break_formation(self):
        """Break the current formation"""
        self.formation_controller.active_formation = None
        self.formation_controller.formation_positions = {}
        self.get_logger().info("Formation broken")
    
    def _send_mission_to_drone(self, drone_id: str, mission: Dict):
        """Send mission to a specific drone's TEE"""
        if drone_id not in self.drone_mission_pubs:
            self.get_logger().error(f"No mission publisher for {drone_id}")
            return
        
        # Convert mission to JSON string
        mission_str = json.dumps(mission)
        msg = String()
        msg.data = mission_str
        
        # Publish to drone's TEE
        self.drone_mission_pubs[drone_id].publish(msg)
        
        self.get_logger().info(
            f"Sent mission {mission.get('mission_id')} to {drone_id}"
        )
    
    def _monitor_squadron_health(self):
        """Monitor squadron health and take corrective actions"""
        unhealthy_drones = self.drone_registry.get_unhealthy_drones()
        
        for drone in unhealthy_drones:
            self.get_logger().warning(
                f"Drone {drone.drone_id} is unhealthy: "
                f"state={drone.state.name}, "
                f"connected={drone.connected}, "
                f"battery={drone.battery_percentage:.1f}%"
            )
            
            # If drone has a task, trigger re-allocation
            if drone.current_task_id:
                mission_id = drone.current_task_id
                
                self.get_logger().warning(
                    f"Drone {drone.drone_id} has active task {mission_id} "
                    f"but is unhealthy - triggering re-allocation"
                )
                
                # Handle as a failure for re-allocation
                self._handle_drone_failure_barriers(drone.drone_id)
                self._handle_mission_failure(
                    drone.drone_id,
                    mission_id,
                    {
                        'state': 'failed',
                        'error': f'Drone unhealthy: {drone.state.name}',
                        'progress': drone.task_progress
                    }
                )
    
    def _publish_squadron_status(self):
        """Publish squadron status"""
        status = self.drone_registry.get_squadron_status()
        
        # Add allocation info
        allocation_summary = self.task_allocator.get_allocation_summary()
        status['allocations'] = allocation_summary
        
        # Publish status
        msg = String()
        msg.data = json.dumps(status)
        self.squadron_status_pub.publish(msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    squadron_manager = SquadronManagerNode()
    
    try:
        rclpy.spin(squadron_manager)
    except KeyboardInterrupt:
        pass
    finally:
        squadron_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
