#!/usr/bin/env python3
"""
Squadron Manager Node - Coordinates multi-drone operations
"""

import json
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Squadron components
from squadron_manager.drone_registry import DroneRegistry, DroneState, DroneCapabilities
from squadron_manager.task_allocator import TaskAllocator, AllocationStrategy, TaskRequirements
from squadron_manager.formation_controller import FormationController, FormationType, FormationParameters

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
            'capability_based': AllocationStrategy.CAPABILITY_BASED
        }
        strategy = strategy_map.get(strategy_str, AllocationStrategy.NEAREST)
        self.task_allocator = TaskAllocator(self.get_logger(), strategy)
        
        self.formation_controller = FormationController(self.get_logger())
        
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
        self.drone_registry.update_drone_connection(
            drone_id=drone_id,
            connected=msg.connected,
            armed=msg.armed,
            mode=msg.mode
        )
    
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
        # NavSatFix status: 0=NO_FIX, 1=FIX, 2=SBAS_FIX, etc.
        fix_type = msg.status.status + 1  # Convert to 1-based
        
        # Extract number of satellites (if available in covariance type)
        # This is simplified - actual implementation may need more info
        satellites = 0  # TODO: Get actual satellite count
        
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
            
            # Update task progress
            if 'progress' in status:
                self.drone_registry.update_task_progress(
                    drone_id=drone_id,
                    progress=status['progress']
                )
            
            # Check if mission completed
            if status.get('state') == 'completed':
                self.drone_registry.complete_task(drone_id)
                self.task_allocator.deallocate_task(mission_id)
                self.get_logger().info(f"✓ Mission {mission_id} completed by {drone_id}")
            
            # Check if mission failed/aborted
            elif status.get('state') in ['aborted', 'failed']:
                self.get_logger().error(
                    f"✗ Mission {mission_id} failed on {drone_id} "
                    f"(reason: {status.get('error', 'unknown')})"
                )
                
                # Handle mission failure with potential re-allocation
                self._handle_mission_failure(drone_id, mission_id, status)
        
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid mission status from {drone_id}")
    
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
        
        # Check if we should attempt re-allocation
        enable_reallocation = self.get_parameter('enable_task_reallocation').value
        
        if enable_reallocation:
            # Get available drones (excluding the failed one)
            available_drones = [
                d for d in self.drone_registry.get_available_drones()
                if d.drone_id != failed_drone_id
            ]
            
            if available_drones:
                self.get_logger().info(
                    f"Attempting to re-allocate failed mission {mission_id} "
                    f"to another drone ({len(available_drones)} available)"
                )
                
                # TODO: Store original mission details for re-allocation
                # For now, just log that re-allocation would happen
                self.get_logger().warning(
                    f"Mission re-allocation not yet fully implemented. "
                    f"Mission {mission_id} lost."
                )
            else:
                self.get_logger().error(
                    f"Cannot re-allocate mission {mission_id}: No available drones"
                )
        else:
            self.get_logger().warning(
                f"Mission re-allocation disabled. Mission {mission_id} lost."
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
        
        # Send mission to selected drone's TEE
        self._send_mission_to_drone(selected_drone_id, mission)
    
    def _handle_multi_drone_mission(self, mission: Dict):
        """Coordinate a multi-drone mission"""
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
        
        # Create formation
        formation_params = FormationParameters(
            formation_type=FormationType(formation_type),
            spacing=mission.get('spacing', 10.0),
            altitude=mission.get('altitude', 50.0)
        )
        
        self.formation_controller.create_formation(available_drones, formation_params)
        
        # Generate individual missions for each drone
        # TODO: Implement waypoint generation based on formation
        
        self.get_logger().info(
            f"Multi-drone mission {mission_id} distributed to {len(available_drones)} drones"
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
            
            # If drone has a task, consider reallocation
            if drone.current_task_id:
                self.get_logger().warning(
                    f"Drone {drone.drone_id} has active task {drone.current_task_id} "
                    f"but is unhealthy - consider reallocation"
                )
                # TODO: Implement task reallocation
    
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
