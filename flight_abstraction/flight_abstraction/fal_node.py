#!/usr/bin/env python3
"""Flight Abstraction Layer (FAL) Node.

This node provides ROS2 action servers for flight primitives and abstracts
MAVROS communication for drone control.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Point

from multi_drone_msgs.action import Takeoff, Land, GoToWaypoint, ExecutePrimitive
from multi_drone_msgs.srv import ArmDisarm
from multi_drone_msgs.msg import DroneStatus

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
    
    def __init__(self, drone_namespace: str = "/drone_0"):
        """
        Initialize the FAL node.
        
        Args:
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
        """
        super().__init__('fal_node')
        
        # Store drone namespace
        self.drone_namespace = drone_namespace
        self.get_logger().info(f"Initializing FAL node for {drone_namespace}")
        
        # Create callback group for concurrent action execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize primitives
        self.arm_primitive = ArmPrimitive(self, drone_namespace)
        self.takeoff_primitive = TakeoffPrimitive(self, drone_namespace)
        self.goto_primitive = GotoPrimitive(self, drone_namespace)
        self.land_primitive = LandPrimitive(self, drone_namespace)
        
        # Create action servers
        self.takeoff_action_server = ActionServer(
            self,
            Takeoff,
            f'{drone_namespace}/takeoff',
            self.takeoff_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.land_action_server = ActionServer(
            self,
            Land,
            f'{drone_namespace}/land',
            self.land_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.goto_action_server = ActionServer(
            self,
            GoToWaypoint,
            f'{drone_namespace}/goto_waypoint',
            self.goto_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        self.execute_primitive_action_server = ActionServer(
            self,
            ExecutePrimitive,
            f'{drone_namespace}/execute_primitive',
            self.execute_primitive_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        
        # Create arm/disarm service
        self.arm_service = self.create_service(
            ArmDisarm,
            f'{drone_namespace}/arm_disarm',
            self.arm_disarm_callback,
            callback_group=self.callback_group
        )
        
        # Create timer for updating primitives
        self.update_timer = self.create_timer(
            0.05,  # 20 Hz
            self.update_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info(f"FAL node initialized successfully")
        self.get_logger().info(f"  - Takeoff action: {drone_namespace}/takeoff")
        self.get_logger().info(f"  - Land action: {drone_namespace}/land")
        self.get_logger().info(f"  - GoToWaypoint action: {drone_namespace}/goto_waypoint")
        self.get_logger().info(f"  - ExecutePrimitive action: {drone_namespace}/execute_primitive")
        self.get_logger().info(f"  - ArmDisarm service: {drone_namespace}/arm_disarm")
    
    def goal_callback(self, goal_request):
        """Accept all goal requests."""
        self.get_logger().info('Received new goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Accept all cancel requests."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def update_callback(self):
        """Update all active primitives."""
        # Update primitives that are executing
        if self.takeoff_primitive.get_state() == PrimitiveState.EXECUTING:
            self.takeoff_primitive.update()
        
        if self.goto_primitive.get_state() == PrimitiveState.EXECUTING:
            self.goto_primitive.update()
        
        if self.land_primitive.get_state() == PrimitiveState.EXECUTING:
            self.land_primitive.update()
    
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
            feedback.progress_percentage = self.takeoff_primitive.get_progress()
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
            result.final_altitude = self.land_primitive.current_altitude
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
                    result.final_altitude = self.land_primitive.current_altitude
                    return result
            
            # Update primitive
            state = self.land_primitive.update()
            
            # Publish feedback
            feedback = Land.Feedback()
            feedback.current_altitude = self.land_primitive.current_altitude
            feedback.progress_percentage = self.land_primitive.get_progress()
            goal_handle.publish_feedback(feedback)
            
            # Check completion
            if state == PrimitiveState.SUCCESS:
                goal_handle.succeed()
                result = Land.Result()
                result.success = True
                result.message = "Landing completed successfully"
                result.final_altitude = self.land_primitive.current_altitude
                return result
            elif state == PrimitiveState.FAILED:
                goal_handle.abort()
                result = Land.Result()
                result.success = False
                result.message = self.land_primitive.get_error_message()
                result.final_altitude = self.land_primitive.current_altitude
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
            result.final_distance_error = self.goto_primitive.get_distance_remaining()
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
                result.final_distance_error = self.goto_primitive.get_distance_remaining()
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
                result.final_distance_error = self.goto_primitive.get_distance_remaining()
                return result
            elif state == PrimitiveState.FAILED:
                goal_handle.abort()
                result = GoToWaypoint.Result()
                result.success = False
                result.message = self.goto_primitive.get_error_message()
                result.final_distance_error = self.goto_primitive.get_distance_remaining()
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
    
    def arm_disarm_callback(self, request, response):
        """Handle arm/disarm service requests."""
        self.get_logger().info(f'Arm/disarm service called: arm={request.arm}')
        
        # Execute arm primitive
        success = self.arm_primitive.execute(arm=request.arm, force=request.force)
        
        if not success:
            response.success = False
            response.message = self.arm_primitive.get_error_message()
            return response
        
        # Wait for completion (with timeout)
        timeout = 5.0
        start_time = self.get_clock().now()
        rate = self.create_rate(10)
        
        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout:
                response.success = False
                response.message = "Arm/disarm timeout"
                return response
            
            state = self.arm_primitive.update()
            
            if state == PrimitiveState.SUCCESS:
                response.success = True
                response.message = f"{'Armed' if request.arm else 'Disarmed'} successfully"
                return response
            elif state == PrimitiveState.FAILED:
                response.success = False
                response.message = self.arm_primitive.get_error_message()
                return response
            
            rate.sleep()
        
        response.success = False
        response.message = "Arm/disarm interrupted"
        return response


def main(args=None):
    """Main entry point for FAL node."""
    rclpy.init(args=args)
    
    # Get drone namespace from command line or use default
    import sys
    drone_namespace = "/drone_0"
    if len(sys.argv) > 1:
        drone_namespace = sys.argv[1]
    
    # Create and spin the FAL node
    fal_node = FALNode(drone_namespace=drone_namespace)
    
    # Use multi-threaded executor for concurrent action execution
    executor = MultiThreadedExecutor()
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
