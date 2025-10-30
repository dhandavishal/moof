#!/usr/bin/env python3
"""
Primitive Command Handler for FAL.

This module receives PrimitiveCommand messages from the Task Execution Engine (TEE)
and converts them into actual FAL primitive instances for execution.

Architecture:
- TEE generates PrimitiveCommand messages (ROS messages)
- PrimitiveCommandHandler subscribes to these messages
- Handler instantiates actual FAL primitive objects (Takeoff, Goto, etc.)
- Handler monitors execution and publishes status back to TEE
"""

import time
import threading
import rclpy
from rclpy.node import Node
from typing import Optional, Dict

from multi_drone_msgs.msg import PrimitiveCommand, PrimitiveStatus
from multi_drone_msgs.action import Takeoff, Land, GoToWaypoint
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup


class PrimitiveCommandHandler(Node):
    """
    Receives PrimitiveCommand messages from TEE and executes them using FAL primitives.
    
    This class bridges the gap between TEE's command messages and FAL's action servers.
    It converts messages into action goals and monitors execution status.
    """
    
    def __init__(self, drone_namespace: str = "/drone_0"):
        """
        Initialize the primitive command handler.
        
        Args:
            drone_namespace: Namespace for this drone (e.g., '/drone_0')
        """
        super().__init__('primitive_command_handler')
        
        self.drone_namespace = drone_namespace
        self.current_command: Optional[PrimitiveCommand] = None
        self.command_in_progress = False
        
        # Create callback group for actions
        self.action_callback_group = ReentrantCallbackGroup()
        
        # Action clients to FAL
        self.takeoff_client = ActionClient(
            self,
            Takeoff,
            f'{drone_namespace}/takeoff',
            callback_group=self.action_callback_group
        )
        
        self.land_client = ActionClient(
            self,
            Land,
            f'{drone_namespace}/land',
            callback_group=self.action_callback_group
        )
        
        self.goto_client = ActionClient(
            self,
            GoToWaypoint,
            f'{drone_namespace}/goto_waypoint',
            callback_group=self.action_callback_group
        )
        
        # Subscribe to primitive commands from TEE
        self.command_sub = self.create_subscription(
            PrimitiveCommand,
            f'{drone_namespace}/primitive_command',
            self._command_callback,
            10
        )
        
        # Publish primitive status back to TEE
        self.status_pub = self.create_publisher(
            PrimitiveStatus,
            f'{drone_namespace}/primitive_status',
            10
        )
        
        self.get_logger().info(f"Primitive command handler initialized for {drone_namespace}")
        self.get_logger().info(f"  - Subscribing to: {drone_namespace}/primitive_command")
        self.get_logger().info(f"  - Publishing to: {drone_namespace}/primitive_status")
    
    def _command_callback(self, cmd: PrimitiveCommand):
        """
        Handle incoming primitive command from TEE.
        
        Args:
            cmd: PrimitiveCommand message
        """
        if self.command_in_progress:
            self.get_logger().warn(
                f"Command {cmd.command_id} received while another is in progress. Queuing not implemented yet."
            )
            self._publish_status(cmd.command_id, 'failed', 0.0, 'Another command is already executing')
            return
        
        self.get_logger().info(
            f"Received primitive command: {cmd.primitive_type} (ID: {cmd.command_id[:8]}...)"
        )
        
        self.current_command = cmd
        self.command_in_progress = True
        
        # Execute primitive based on type
        try:
            if cmd.primitive_type == 'goto':
                self._execute_goto(cmd)
            
            elif cmd.primitive_type == 'takeoff':
                self._execute_takeoff(cmd)
            
            elif cmd.primitive_type == 'land':
                self._execute_land(cmd)
            
            elif cmd.primitive_type == 'loiter':
                self.get_logger().warn(f"Loiter primitive not yet implemented")
                self._publish_status(cmd.command_id, 'failed', 0.0, 'Loiter not implemented')
                self.command_in_progress = False
            
            elif cmd.primitive_type == 'rtl':
                self.get_logger().warn(f"RTL primitive not yet implemented")
                self._publish_status(cmd.command_id, 'failed', 0.0, 'RTL not implemented')
                self.command_in_progress = False
            
            else:
                self.get_logger().error(f"Unknown primitive type: {cmd.primitive_type}")
                self._publish_status(cmd.command_id, 'failed', 0.0, f'Unknown primitive type: {cmd.primitive_type}')
                self.command_in_progress = False
        
        except Exception as e:
            self.get_logger().error(f"Failed to execute primitive: {e}")
            self._publish_status(cmd.command_id, 'failed', 0.0, str(e))
            self.command_in_progress = False
    
    def _execute_takeoff(self, cmd: PrimitiveCommand):
        """Execute takeoff primitive via FAL action."""
        self.get_logger().info(f"Executing takeoff to {cmd.target_position.z}m")
        
        # Wait for action server
        if not self.takeoff_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Takeoff action server not available")
            self._publish_status(cmd.command_id, 'failed', 0.0, 'Takeoff action server timeout')
            self.command_in_progress = False
            return
        
        # Create goal
        goal_msg = Takeoff.Goal()
        goal_msg.target_altitude = cmd.target_position.z
        goal_msg.climb_rate = cmd.climb_rate if cmd.climb_rate > 0 else 1.0
        
        # Send goal
        self._publish_status(cmd.command_id, 'executing', 0.0)
        
        send_goal_future = self.takeoff_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg: self._takeoff_feedback_callback(cmd.command_id, feedback_msg)
        )
        send_goal_future.add_done_callback(lambda future: self._goal_response_callback(cmd, future, 'takeoff'))
    
    def _execute_land(self, cmd: PrimitiveCommand):
        """Execute land primitive via FAL action."""
        self.get_logger().info(f"Executing land")
        
        # Wait for action server
        if not self.land_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Land action server not available")
            self._publish_status(cmd.command_id, 'failed', 0.0, 'Land action server timeout')
            self.command_in_progress = False
            return
        
        # Create goal
        goal_msg = Land.Goal()
        goal_msg.target_position = cmd.target_position
        goal_msg.descent_rate = cmd.climb_rate if cmd.climb_rate > 0 else 0.5
        goal_msg.precision_land = False
        
        # Send goal
        self._publish_status(cmd.command_id, 'executing', 0.0)
        
        send_goal_future = self.land_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg: self._land_feedback_callback(cmd.command_id, feedback_msg)
        )
        send_goal_future.add_done_callback(lambda future: self._goal_response_callback(cmd, future, 'land'))
    
    def _execute_goto(self, cmd: PrimitiveCommand):
        """Execute goto primitive via FAL action."""
        self.get_logger().info(
            f"Executing goto to ({cmd.target_position.x:.1f}, {cmd.target_position.y:.1f}, {cmd.target_position.z:.1f})"
        )
        
        # Wait for action server
        if not self.goto_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Goto action server not available")
            self._publish_status(cmd.command_id, 'failed', 0.0, 'Goto action server timeout')
            self.command_in_progress = False
            return
        
        # Create goal
        goal_msg = GoToWaypoint.Goal()
        goal_msg.target_position = cmd.target_position
        goal_msg.max_speed = cmd.velocity if cmd.velocity > 0 else 2.0
        goal_msg.acceptance_radius = cmd.acceptance_radius if cmd.acceptance_radius > 0 else 1.0
        goal_msg.target_heading = 0.0  # TODO: Extract from quaternion if needed
        
        # Send goal
        self._publish_status(cmd.command_id, 'executing', 0.0)
        
        send_goal_future = self.goto_client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback_msg: self._goto_feedback_callback(cmd.command_id, feedback_msg)
        )
        send_goal_future.add_done_callback(lambda future: self._goal_response_callback(cmd, future, 'goto'))
    
    def _goal_response_callback(self, cmd: PrimitiveCommand, future, primitive_type: str):
        """Handle goal response and wait for result."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f"{primitive_type} goal rejected")
            self._publish_status(cmd.command_id, 'failed', 0.0, f'{primitive_type} goal rejected')
            self.command_in_progress = False
            return
        
        self.get_logger().info(f"{primitive_type} goal accepted")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda future: self._result_callback(cmd, future, primitive_type))
    
    def _result_callback(self, cmd: PrimitiveCommand, future, primitive_type: str):
        """Handle action result."""
        result = future.result().result
        
        if result.success:
            self.get_logger().info(f"{primitive_type} completed: {result.message}")
            self._publish_status(cmd.command_id, 'completed', 1.0)
        else:
            self.get_logger().error(f"{primitive_type} failed: {result.message}")
            self._publish_status(cmd.command_id, 'failed', 1.0, result.message)
        
        self.command_in_progress = False
        self.current_command = None
    
    def _takeoff_feedback_callback(self, command_id: str, feedback_msg):
        """Handle takeoff feedback."""
        feedback = feedback_msg.feedback
        progress = feedback.progress / 100.0  # Convert 0-100 to 0.0-1.0
        self._publish_status(command_id, 'executing', progress)
    
    def _land_feedback_callback(self, command_id: str, feedback_msg):
        """Handle land feedback."""
        feedback = feedback_msg.feedback
        progress = feedback.progress / 100.0  # Convert 0-100 to 0.0-1.0
        self._publish_status(command_id, 'executing', progress)
    
    def _goto_feedback_callback(self, command_id: str, feedback_msg):
        """Handle goto feedback."""
        feedback = feedback_msg.feedback
        # For goto, we don't have direct progress, so estimate from distance
        # This is a rough estimate - could be improved
        self._publish_status(command_id, 'executing', 0.5)  # Mid-progress estimate
    
    def _publish_status(self, 
                       command_id: str, 
                       status: str, 
                       progress: float = 0.0,
                       error_message: str = ''):
        """
        Publish primitive status to TEE.
        
        Args:
            command_id: Command identifier
            status: Status string (executing, completed, failed, timeout, cancelled)
            progress: Progress 0.0-1.0
            error_message: Error message if failed
        """
        msg = PrimitiveStatus()
        msg.command_id = command_id
        msg.status = status
        msg.progress = progress
        msg.error_message = error_message
        msg.timestamp = self.get_clock().now().to_msg()
        
        self.status_pub.publish(msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Get drone namespace from command line or use default
    import sys
    drone_namespace = "/drone_0"
    if len(sys.argv) > 1:
        drone_namespace = sys.argv[1]
    
    handler = PrimitiveCommandHandler(drone_namespace=drone_namespace)
    
    try:
        rclpy.spin(handler)
    except KeyboardInterrupt:
        pass
    finally:
        handler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
