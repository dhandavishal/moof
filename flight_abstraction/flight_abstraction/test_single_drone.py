#!/usr/bin/env python3
"""
Single drone FAL test script.

This script tests the complete flight sequence:
1. Arm
2. Takeoff to 10m
3. Navigate to waypoint (10, 0, 10)
4. Navigate to waypoint (10, 10, 10)
5. Navigate to waypoint (0, 10, 10)
6. Navigate back to (0, 0, 10)
7. Land

Usage:
    ros2 run flight_abstraction test_single_drone
    
    Or with specific drone:
    ros2 run flight_abstraction test_single_drone --ros-args -p drone_id:=0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from multi_drone_msgs.action import Takeoff, Land, GoToWaypoint
from multi_drone_msgs.srv import ArmDisarm
from geometry_msgs.msg import Point
import time


class SingleDroneTest(Node):
    """Test node for single drone FAL operations."""
    
    def __init__(self):
        super().__init__('single_drone_test')
        
        # Declare parameter
        self.declare_parameter('drone_id', 0)
        drone_id = self.get_parameter('drone_id').value
        self.drone_namespace = f'/drone_{drone_id}'
        
        self.get_logger().info(f'Testing FAL for {self.drone_namespace}')
        
        # Create service client for arm/disarm
        self.arm_client = self.create_client(
            ArmDisarm,
            f'{self.drone_namespace}/arm_disarm'
        )
        
        # Create action clients
        self.takeoff_client = ActionClient(
            self,
            Takeoff,
            f'{self.drone_namespace}/takeoff'
        )
        
        self.goto_client = ActionClient(
            self,
            GoToWaypoint,
            f'{self.drone_namespace}/goto_waypoint'
        )
        
        self.land_client = ActionClient(
            self,
            Land,
            f'{self.drone_namespace}/land'
        )
        
        # Test state
        self.test_passed = True
        self.test_results = []
        
    def wait_for_services(self, timeout=10.0):
        """Wait for all services and action servers."""
        self.get_logger().info('Waiting for FAL services and actions...')
        
        # Wait for arm service
        if not self.arm_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('Arm/disarm service not available')
            return False
        
        # Wait for action servers
        if not self.takeoff_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Takeoff action server not available')
            return False
        
        if not self.goto_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('GoTo action server not available')
            return False
        
        if not self.land_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('Land action server not available')
            return False
        
        self.get_logger().info('All FAL services and actions available')
        return True
    
    def arm(self, arm=True):
        """Arm or disarm the drone."""
        test_name = 'Arm' if arm else 'Disarm'
        self.get_logger().info(f'{test_name}ing drone...')
        
        request = ArmDisarm.Request()
        request.arm = arm
        request.force = False
        
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{test_name} successful: {response.message}')
                self.test_results.append((test_name, True, response.message))
                return True
            else:
                self.get_logger().error(f'{test_name} failed: {response.message}')
                self.test_results.append((test_name, False, response.message))
                self.test_passed = False
                return False
        else:
            self.get_logger().error(f'{test_name} service call timed out')
            self.test_results.append((test_name, False, 'Timeout'))
            self.test_passed = False
            return False
    
    def takeoff(self, altitude=10.0):
        """Execute takeoff."""
        self.get_logger().info(f'Taking off to {altitude}m...')
        
        goal = Takeoff.Goal()
        goal.target_altitude = altitude
        goal.climb_rate = 1.0
        
        send_future = self.takeoff_client.send_goal_async(
            goal,
            feedback_callback=self._takeoff_feedback
        )
        
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if send_future.result() is None:
            self.get_logger().error('Takeoff goal rejected or timed out')
            self.test_results.append(('Takeoff', False, 'Goal rejected'))
            self.test_passed = False
            return False
        
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Takeoff goal rejected')
            self.test_results.append(('Takeoff', False, 'Goal rejected'))
            self.test_passed = False
            return False
        
        self.get_logger().info('Takeoff goal accepted, waiting for completion...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        
        if result_future.result() is None:
            self.get_logger().error('Takeoff result timed out')
            self.test_results.append(('Takeoff', False, 'Result timeout'))
            self.test_passed = False
            return False
        
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f'Takeoff successful: {result.message}')
            self.test_results.append(('Takeoff', True, f'Altitude: {result.final_altitude:.2f}m'))
            return True
        else:
            self.get_logger().error(f'Takeoff failed: {result.message}')
            self.test_results.append(('Takeoff', False, result.message))
            self.test_passed = False
            return False
    
    def _takeoff_feedback(self, feedback_msg):
        """Callback for takeoff feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  Takeoff: altitude={feedback.current_altitude:.2f}m, '
            f'progress={feedback.progress:.1f}%',
            throttle_duration_sec=2.0
        )
    
    def goto_waypoint(self, x, y, z, heading=0.0, speed=2.0, radius=1.0):
        """Navigate to waypoint."""
        self.get_logger().info(f'Going to waypoint ({x:.1f}, {y:.1f}, {z:.1f})...')
        
        goal = GoToWaypoint.Goal()
        goal.target_position = Point(x=x, y=y, z=z)
        goal.target_heading = heading
        goal.max_speed = speed
        goal.acceptance_radius = radius
        
        send_future = self.goto_client.send_goal_async(
            goal,
            feedback_callback=self._goto_feedback
        )
        
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if send_future.result() is None:
            self.get_logger().error('Goto goal rejected or timed out')
            self.test_results.append((f'Goto ({x},{y},{z})', False, 'Goal rejected'))
            self.test_passed = False
            return False
        
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goto goal rejected')
            self.test_results.append((f'Goto ({x},{y},{z})', False, 'Goal rejected'))
            self.test_passed = False
            return False
        
        self.get_logger().info('Goto goal accepted, waiting for completion...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        
        if result_future.result() is None:
            self.get_logger().error('Goto result timed out')
            self.test_results.append((f'Goto ({x},{y},{z})', False, 'Result timeout'))
            self.test_passed = False
            return False
        
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f'Waypoint reached: {result.message}')
            self.test_results.append((f'Goto ({x},{y},{z})', True, f'Error: {result.distance_error:.2f}m'))
            return True
        else:
            self.get_logger().error(f'Goto failed: {result.message}')
            self.test_results.append((f'Goto ({x},{y},{z})', False, result.message))
            self.test_passed = False
            return False
    
    def _goto_feedback(self, feedback_msg):
        """Callback for goto feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'  Goto: distance={feedback.distance_remaining:.2f}m, '
            f'ETA={feedback.estimated_time_remaining:.1f}s',
            throttle_duration_sec=2.0
        )
    
    def land(self):
        """Execute landing."""
        self.get_logger().info('Landing...')
        
        goal = Land.Goal()
        goal.descent_rate = 0.5
        
        send_future = self.land_client.send_goal_async(
            goal,
            feedback_callback=self._land_feedback
        )
        
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        
        if send_future.result() is None:
            self.get_logger().error('Land goal rejected or timed out')
            self.test_results.append(('Land', False, 'Goal rejected'))
            self.test_passed = False
            return False
        
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Land goal rejected')
            self.test_results.append(('Land', False, 'Goal rejected'))
            self.test_passed = False
            return False
        
        self.get_logger().info('Land goal accepted, waiting for completion...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)
        
        if result_future.result() is None:
            self.get_logger().error('Land result timed out')
            self.test_results.append(('Land', False, 'Result timeout'))
            self.test_passed = False
            return False
        
        result = result_future.result().result
        
        if result.success:
            self.get_logger().info(f'Landing successful: {result.message}')
            self.test_results.append(('Land', True, f'Position: ({result.final_position.x:.2f}, {result.final_position.y:.2f}, {result.final_position.z:.2f})'))
            return True
        else:
            self.get_logger().error(f'Landing failed: {result.message}')
            self.test_results.append(('Land', False, result.message))
            self.test_passed = False
            return False
    
    def _land_feedback(self, feedback_msg):
        """Handle land action feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Landing: {feedback.current_altitude:.2f}m, '
            f'Progress: {feedback.progress:.1f}%, '
            f'Status: {feedback.status}'
        )
    
    def print_summary(self):
        """Print test summary."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('TEST SUMMARY')
        self.get_logger().info('=' * 60)
        
        for test_name, passed, details in self.test_results:
            status = '✓ PASS' if passed else '✗ FAIL'
            self.get_logger().info(f'{status} - {test_name}: {details}')
        
        self.get_logger().info('=' * 60)
        if self.test_passed:
            self.get_logger().info('✓ ALL TESTS PASSED')
        else:
            self.get_logger().error('✗ SOME TESTS FAILED')
        self.get_logger().info('=' * 60)
    
    def run_test(self):
        """Run complete test sequence."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING SINGLE DRONE FAL TEST')
        self.get_logger().info('=' * 60)
        
        # Wait for services
        if not self.wait_for_services():
            self.get_logger().error('Failed to connect to FAL services')
            return False
        
        time.sleep(2)  # Give time for everything to stabilize
        
        # Test sequence
        try:
            # 1. Arm
            if not self.arm(arm=True):
                self.get_logger().error('Arm failed, aborting test')
                return False
            
            time.sleep(2)
            
            # 2. Takeoff
            if not self.takeoff(altitude=10.0):
                self.get_logger().error('Takeoff failed, aborting test')
                return False
            
            time.sleep(2)
            
            # 3. Navigate in square pattern
            waypoints = [
                (10.0, 0.0, 10.0),
                (10.0, 10.0, 10.0),
                (0.0, 10.0, 10.0),
                (0.0, 0.0, 10.0),
            ]
            
            for x, y, z in waypoints:
                if not self.goto_waypoint(x, y, z):
                    self.get_logger().error(f'Goto waypoint ({x},{y},{z}) failed')
                    # Continue anyway for testing
                time.sleep(1)
            
            # 4. Land
            if not self.land():
                self.get_logger().error('Landing failed')
                return False
            
            time.sleep(2)
            
            # 5. Disarm
            self.arm(arm=False)
            
            # Print summary
            self.print_summary()
            
            return self.test_passed
            
        except Exception as e:
            self.get_logger().error(f'Test exception: {str(e)}')
            return False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    test_node = SingleDroneTest()
    
    try:
        success = test_node.run_test()
        
        # Keep node alive briefly for final messages
        time.sleep(2)
        
        # Exit with appropriate code
        exit(0 if success else 1)
        
    except KeyboardInterrupt:
        test_node.get_logger().info('Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
