#!/usr/bin/env python3
"""Flight Abstraction Layer (FAL) Node - Fixed Version.

This node provides ROS2 action servers for flight primitives and abstracts
MAVROS communication for drone control.
"""

import os
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import Point

from multi_drone_msgs.action import Takeoff, Land, GoToWaypoint, ExecutePrimitive
from multi_drone_msgs.srv import ArmDisarm
from multi_drone_msgs.msg import DroneStatus, PrimitiveCommand, PrimitiveStatus

from flight_abstraction.primitives import (
    ArmPrimitive,
    TakeoffPrimitive,
    GotoPrimitive,
    LandPrimitive,
    PrimitiveState
)


class FALNode(Node):
    """
    Flight Abstraction Layer Node.
    
    Provides action servers for flight primitives and manages drone control
    through MAVROS.
    """
    
    def __init__(self, drone_namespace: str = None):
        """
        Initialize the FAL node.
        
        Args:
            drone_namespace: Namespace for this drone (e.g., '/drone_0').
                           If None, derives from ROS2 node namespace.
        """
        super().__init__('fal_node')
        
        # Derive drone namespace from ROS2 node namespace if not provided
        if drone_namespace is None:
            node_ns = self.get_namespace()
            self.drone_namespace = node_ns if node_ns != '/' else '/drone_0'
        else:
            self.drone_namespace = drone_namespace
            
        self.get_logger().info(f"Initializing FAL node for {self.drone_namespace}")
        
        # Create callback groups - all Reentrant to allow concurrent execution
        self.action_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()
        self.timer_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = ReentrantCallbackGroup()
        
        # Initialize primitives with shared subscription callback group
        self.arm_primitive = ArmPrimitive(self, self.drone_namespace, self.subscription_callback_group)
        self.takeoff_primitive = TakeoffPrimitive(self, self.drone_namespace, self.subscription_callback_group)
        self.goto_primitive = GotoPrimitive(self, self.drone_namespace, self.subscription_callback_group)
        self.land_primitive = LandPrimitive(self, self.drone_namespace, self.subscription_callback_group)
        
        # Create action servers
        self.takeoff_action_server = ActionServer(
            self,
            Takeoff,
            f'{self.drone_namespace}/takeoff',
            self.takeoff_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )
        
        self.land_action_server = ActionServer(
            self,
            Land,
            f'{self.drone_namespace}/land',
            self.land_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )
        
        self.goto_action_server = ActionServer(
            self,
            GoToWaypoint,
            f'{self.drone_namespace}/goto_waypoint',
            self.goto_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )
        
        self.execute_primitive_action_server = ActionServer(
            self,
            ExecutePrimitive,
            f'{self.drone_namespace}/execute_primitive',
            self.execute_primitive_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.action_callback_group
        )
        
        # Create arm/disarm service with async handling
        self.arm_service = self.create_service(
            ArmDisarm,
            f'{self.drone_namespace}/arm_disarm',
            self.arm_disarm_callback_async,
            callback_group=self.service_callback_group
        )
        
        # Topic-based communication with TEE
        # Subscribe to primitive commands from TEE
        self.primitive_command_sub = self.create_subscription(
            PrimitiveCommand,
            '/tee/primitive_command',
            self.primitive_command_callback,
            10,
            callback_group=self.subscription_callback_group
        )
        
        # Publish primitive status to TEE
        self.primitive_status_pub = self.create_publisher(
            PrimitiveStatus,
            '/fal/primitive_status',
            10
        )
        
        # Track currently executing topic-based primitive
        self.current_topic_primitive = None
        self.current_primitive_id = None
        
        # Create timer for updating primitives
        self.update_timer = self.create_timer(
            0.05,  # 20 Hz
            self.update_callback,
            callback_group=self.timer_callback_group
        )
        
        self.get_logger().info(f"FAL node initialized successfully")
        self.get_logger().info(f"  - Takeoff action: {drone_namespace}/takeoff")
        self.get_logger().info(f"  - Land action: {drone_namespace}/land")
        self.get_logger().info(f"  - GoToWaypoint action: {drone_namespace}/goto_waypoint")
        self.get_logger().info(f"  - ExecutePrimitive action: {drone_namespace}/execute_primitive")
        self.get_logger().info(f"  - ArmDisarm service: {drone_namespace}/arm_disarm")
        self.get_logger().info(f"  - Subscribed to: /tee/primitive_command")
        self.get_logger().info(f"  - Publishing to: /fal/primitive_status")
    
    def goal_callback(self, goal_request):
        """Accept all goal requests."""
        self.get_logger().info(f'Received new goal request: {type(goal_request).__name__}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def update_callback(self):
        """Update all active primitives."""
        # Update arm primitive if executing (check this first as it's fast)
        if self.arm_primitive.get_state() == PrimitiveState.EXECUTING:
            self.arm_primitive.update()
            
        # Update other primitives that are executing
        if self.takeoff_primitive.get_state() == PrimitiveState.EXECUTING:
            self.takeoff_primitive.update()
        
        if self.goto_primitive.get_state() == PrimitiveState.EXECUTING:
            self.goto_primitive.update()
        
        if self.land_primitive.get_state() == PrimitiveState.EXECUTING:
            self.land_primitive.update()
        
        # Check topic-based primitive status and publish updates
        if self.current_topic_primitive is not None:
            state = self.current_topic_primitive.get_state()
            status_msg = PrimitiveStatus()
            status_msg.command_id = self.current_primitive_id
            status_msg.progress = 0.5 if state == PrimitiveState.EXECUTING else 0.0
            
            if state == PrimitiveState.IDLE:
                status_msg.status = 'idle'
                status_msg.progress = 0.0
            elif state == PrimitiveState.EXECUTING:
                status_msg.status = 'executing'
                status_msg.progress = 0.5
            elif state == PrimitiveState.SUCCESS:
                status_msg.status = 'completed'
                status_msg.progress = 1.0
                status_msg.error_message = 'Primitive completed successfully'
                self.primitive_status_pub.publish(status_msg)
                self.get_logger().info(f"Primitive {self.current_primitive_id} completed successfully")
                self.current_topic_primitive = None
                self.current_primitive_id = None
            elif state == PrimitiveState.FAILED:
                status_msg.status = 'failed'
                status_msg.progress = 0.0
                status_msg.error_message = self.current_topic_primitive.get_error_message()
                self.primitive_status_pub.publish(status_msg)
                self.get_logger().error(f"Primitive {self.current_primitive_id} failed: {status_msg.error_message}")
                self.current_topic_primitive = None
                self.current_primitive_id = None
            else:
                status_msg.status = 'unknown'
                status_msg.progress = 0.0
            
            # Publish status updates for executing primitives
            if state == PrimitiveState.EXECUTING:
                self.primitive_status_pub.publish(status_msg)
    
    def _get_primitive_type_string(self, primitive):
        """Get string representation of primitive type."""
        if isinstance(primitive, ArmPrimitive):
            return 'arm'
        elif isinstance(primitive, TakeoffPrimitive):
            return 'takeoff'
        elif isinstance(primitive, GotoPrimitive):
            return 'goto'
        elif isinstance(primitive, LandPrimitive):
            return 'land'
        else:
            return 'unknown'
    
    def primitive_command_callback(self, msg: PrimitiveCommand):
        """Handle primitive commands from TEE via topic."""
        self.get_logger().info(f"Received primitive command: {msg.primitive_type} (ID: {msg.command_id})")
        
        # Check if we're already executing a primitive
        if self.current_topic_primitive is not None:
            current_state = self.current_topic_primitive.get_state()
            if current_state == PrimitiveState.EXECUTING:
                self.get_logger().warn(f"Ignoring command - primitive {self.current_primitive_id} still executing")
                status_msg = PrimitiveStatus()
                status_msg.command_id = msg.command_id
                status_msg.status = 'rejected'
                status_msg.error_message = f'Already executing primitive {self.current_primitive_id}'
                self.primitive_status_pub.publish(status_msg)
                return
        
        # Store primitive ID for tracking
        self.current_primitive_id = msg.command_id
        
        # Execute the appropriate primitive
        success = False
        
        if msg.primitive_type == 'arm':
            self.current_topic_primitive = self.arm_primitive
            # For arm: check if altitude > 0 means arm, else disarm
            arm_value = True  # Default to arm
            success = self.arm_primitive.execute(arm=arm_value, force=False, timeout=10.0)
            
        elif msg.primitive_type == 'disarm':
            self.current_topic_primitive = self.arm_primitive
            success = self.arm_primitive.execute(arm=False, force=False, timeout=10.0)
            
        elif msg.primitive_type == 'takeoff':
            self.current_topic_primitive = self.takeoff_primitive
            # Use target_position.z as altitude, climb_rate from message
            target_altitude = msg.target_position.z if msg.target_position.z > 0 else 10.0
            climb_rate = msg.climb_rate if msg.climb_rate > 0 else 1.0
            success = self.takeoff_primitive.execute(target_altitude=target_altitude, climb_rate=climb_rate, timeout=msg.timeout if msg.timeout > 0 else 30.0)
            
        elif msg.primitive_type == 'goto':
            self.current_topic_primitive = self.goto_primitive
            # Use target_position directly
            success = self.goto_primitive.execute(target=msg.target_position, timeout=msg.timeout if msg.timeout > 0 else 60.0)
                
        elif msg.primitive_type == 'land':
            self.current_topic_primitive = self.land_primitive
            success = self.land_primitive.execute(timeout=msg.timeout if msg.timeout > 0 else 30.0)
            
        else:
            self.get_logger().error(f"Unknown primitive type: {msg.primitive_type}")
        
        # Publish initial status
        status_msg = PrimitiveStatus()
        status_msg.command_id = msg.command_id
        status_msg.progress = 0.0
        
        if success:
            status_msg.status = 'executing'
            status_msg.error_message = f'Started {msg.primitive_type} primitive'
            self.get_logger().info(f"Primitive {msg.command_id} started successfully")
        else:
            status_msg.status = 'failed'
            status_msg.error_message = f'Failed to start {msg.primitive_type}: {self.current_topic_primitive.get_error_message() if self.current_topic_primitive else "Unknown error"}'
            self.get_logger().error(f"Failed to start primitive {msg.command_id}: {status_msg.error_message}")
            self.current_topic_primitive = None
            self.current_primitive_id = None
        
        self.primitive_status_pub.publish(status_msg)
    
    def arm_disarm_callback_async(self, request, response):
        """Handle arm/disarm service requests with proper async handling."""
        self.get_logger().info(f'Arm/disarm service called: arm={request.arm}')
        
        # Execute arm primitive (this sends the command and returns immediately)
        # The main update_timer will now handle polling
        success = self.arm_primitive.execute(arm=request.arm, force=request.force, timeout=10.0)
        
        if not success:
            response.success = False
            response.message = self.arm_primitive.get_error_message()
            response.current_armed_state = self.arm_primitive.current_state.armed if self.arm_primitive.current_state else False
            self.get_logger().error(f"Arm primitive execute failed: {response.message}")
            return response
        
        # Wait for completion using a non-blocking approach
        timeout = 15.0  # Total timeout
        check_interval = 0.1  # Check every 100ms
        start_time = time.time()
        
        self.get_logger().info(f"Waiting for arm/disarm completion (timeout={timeout}s)...")
        
        while time.time() - start_time < timeout:
            # We just check the state, which is updated by the main update_timer callback
            state = self.arm_primitive.get_state()
            
            # Check for completion
            if state == PrimitiveState.SUCCESS:
                response.success = True
                response.message = f"{'Armed' if request.arm else 'Disarmed'} successfully"
                response.current_armed_state = self.arm_primitive.current_state.armed if self.arm_primitive.current_state else False
                self.get_logger().info(response.message)
                return response
            elif state == PrimitiveState.FAILED:
                response.success = False
                response.message = self.arm_primitive.get_error_message()
                response.current_armed_state = self.arm_primitive.current_state.armed if self.arm_primitive.current_state else False
                self.get_logger().error(f"Arm primitive failed: {response.message}")
                return response
            
            # Sleep briefly to yield the thread, allowing other callbacks to run
            # DO NOT SPIN. Just sleep.
            time.sleep(check_interval)
        
        # If we get here, it timed out
        response.success = False
        response.message = f"Arm/disarm timeout after {timeout}s"
        response.current_armed_state = self.arm_primitive.current_state.armed if self.arm_primitive.current_state else False
        self.get_logger().error(response.message)
        return response
    
    async def takeoff_execute_callback(self, goal_handle):
        """Execute takeoff action."""
        self.get_logger().info(f'Executing takeoff to {goal_handle.request.target_altitude}m')
        
        # Execute the primitive
        success = self.takeoff_primitive.execute(
            target_altitude=goal_handle.request.target_altitude,
            climb_rate=goal_handle.request.climb_rate,
            timeout=60.0
        )
        
        if not success:
            goal_handle.abort()
            result = Takeoff.Result()
            result.success = False
            result.message = self.takeoff_primitive.get_error_message()
            result.final_altitude = self.takeoff_primitive.current_altitude
            return result
        
        # Monitor progress
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            # Check if cancelled
            if goal_handle.is_cancel_requested:
                self.takeoff_primitive.cancel()
                goal_handle.canceled()
                result = Takeoff.Result()
                result.success = False
                result.message = "Takeoff cancelled"
                result.final_altitude = self.takeoff_primitive.current_altitude
                return result
            
            # Update primitive
            state = self.takeoff_primitive.update()
            
            # Publish feedback
            feedback = Takeoff.Feedback()
            feedback.current_altitude = self.takeoff_primitive.current_altitude
            feedback.progress = self.takeoff_primitive.get_progress()
            goal_handle.publish_feedback(feedback)
            
            # Check completion
            if state == PrimitiveState.SUCCESS:
                goal_handle.succeed()
                result = Takeoff.Result()
                result.success = True
                result.message = "Takeoff completed successfully"
                result.final_altitude = self.takeoff_primitive.current_altitude
                return result
            elif state == PrimitiveState.FAILED:
                goal_handle.abort()
                result = Takeoff.Result()
                result.success = False
                result.message = self.takeoff_primitive.get_error_message()
                result.final_altitude = self.takeoff_primitive.current_altitude
                return result
            
            rate.sleep()
    
    async def land_execute_callback(self, goal_handle):
        """Execute land action."""
        self.get_logger().info('Executing landing')
        
        # Get target position if provided
        target_position = None
        if hasattr(goal_handle.request, 'target_position'):
            target_position = goal_handle.request.target_position
        
        # Execute the primitive
        success = self.land_primitive.execute(
            target_position=target_position,
            descent_rate=goal_handle.request.descent_rate if hasattr(goal_handle.request, 'descent_rate') else 0.5,
            timeout=60.0
        )
        
        if not success:
            goal_handle.abort()
            result = Land.Result()
            result.success = False
            result.message = self.land_primitive.get_error_message()
            result.final_position = self.land_primitive.current_position
            return result
        
        # Monitor progress
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            # Check if cancelled
            if goal_handle.is_cancel_requested:
                if self.land_primitive.cancel():
                    goal_handle.canceled()
                    result = Land.Result()
                    result.success = False
                    result.message = "Landing cancelled"
                    result.final_position = self.land_primitive.current_position
                    return result
            
            # Update primitive
            state = self.land_primitive.update()
            
            # Publish feedback
            feedback = Land.Feedback()
            feedback.current_altitude = self.land_primitive.current_altitude
            feedback.progress = self.land_primitive.get_progress()
            goal_handle.publish_feedback(feedback)
            
            # Check completion
            if state == PrimitiveState.SUCCESS:
                goal_handle.succeed()
                result = Land.Result()
                result.success = True
                result.message = "Landing completed successfully"
                result.final_position = self.land_primitive.current_position
                return result
            elif state == PrimitiveState.FAILED:
                goal_handle.abort()
                result = Land.Result()
                result.success = False
                result.message = self.land_primitive.get_error_message()
                result.final_position = self.land_primitive.current_position
                return result
            
            rate.sleep()
    
    async def goto_execute_callback(self, goal_handle):
        """Execute goto waypoint action."""
        self.get_logger().info(f'Executing goto waypoint: ({goal_handle.request.target_position.x}, '
                              f'{goal_handle.request.target_position.y}, {goal_handle.request.target_position.z})')
        
        # Execute the primitive
        success = self.goto_primitive.execute(
            target_position=goal_handle.request.target_position,
            target_heading=goal_handle.request.target_heading,
            max_speed=goal_handle.request.max_speed,
            acceptance_radius=goal_handle.request.acceptance_radius,
            timeout=120.0
        )
        
        if not success:
            goal_handle.abort()
            result = GoToWaypoint.Result()
            result.success = False
            result.message = self.goto_primitive.get_error_message()
            result.distance_error = self.goto_primitive.get_distance_remaining()
            return result
        
        # Monitor progress
        rate = self.create_rate(10)  # 10 Hz
        while rclpy.ok():
            # Check if cancelled
            if goal_handle.is_cancel_requested:
                self.goto_primitive.cancel()
                goal_handle.canceled()
                result = GoToWaypoint.Result()
                result.success = False
                result.message = "Goto cancelled"
                result.distance_error = self.goto_primitive.get_distance_remaining()
                return result
            
            # Update primitive
            state = self.goto_primitive.update()
            
            # Publish feedback
            feedback = GoToWaypoint.Feedback()
            feedback.distance_remaining = self.goto_primitive.get_distance_remaining()
            feedback.estimated_time_remaining = self.goto_primitive.get_estimated_time_remaining()
            feedback.current_position = self.goto_primitive.current_position
            goal_handle.publish_feedback(feedback)
            
            # Check completion
            if state == PrimitiveState.SUCCESS:
                goal_handle.succeed()
                result = GoToWaypoint.Result()
                result.success = True
                result.message = "Waypoint reached successfully"
                result.distance_error = self.goto_primitive.get_distance_remaining()
                return result
            elif state == PrimitiveState.FAILED:
                goal_handle.abort()
                result = GoToWaypoint.Result()
                result.success = False
                result.message = self.goto_primitive.get_error_message()
                result.distance_error = self.goto_primitive.get_distance_remaining()
                return result
            
            rate.sleep()
    
    async def execute_primitive_callback(self, goal_handle):
        """Execute generic primitive action."""
        primitive_type = goal_handle.request.primitive_type
        self.get_logger().info(f'Executing primitive: {primitive_type}')
        
        # Route to specific primitive based on type
        if primitive_type == "arm":
            # Parse parameters
            arm_value = True
            if len(goal_handle.request.parameters) > 0:
                arm_value = goal_handle.request.parameters[0] != "0"
            
            success = self.arm_primitive.execute(arm=arm_value)
            
            # Wait for completion
            rate = self.create_rate(10)
            while rclpy.ok():
                state = self.arm_primitive.update()
                
                feedback = ExecutePrimitive.Feedback()
                feedback.progress_percentage = self.arm_primitive.get_progress()
                feedback.status_message = f"Arming state: {state.name}"
                goal_handle.publish_feedback(feedback)
                
                if state in [PrimitiveState.SUCCESS, PrimitiveState.FAILED]:
                    break
                
                rate.sleep()
            
            result = ExecutePrimitive.Result()
            result.success = (state == PrimitiveState.SUCCESS)
            result.message = self.arm_primitive.get_error_message() if not result.success else "Arm/disarm successful"
            
            if result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            
            return result
        else:
            goal_handle.abort()
            result = ExecutePrimitive.Result()
            result.success = False
            result.message = f"Unknown primitive type: {primitive_type}"
            return result


def main(args=None):
    """Main entry point for FAL node."""
    rclpy.init(args=args)
    
    # Create FAL node first - it will inherit namespace from launch
    # Then determine drone_namespace from node's own namespace
    fal_node = FALNode(drone_namespace=None)  # Will be set in __init__
    
    # Use multi-threaded executor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(fal_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        fal_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()