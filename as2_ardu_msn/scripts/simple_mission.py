#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import YawMode
import time

class SimpleMission(Node):
    def __init__(self):
        super().__init__('simple_mission')
        
        # Initialize drone interface
        self.drone = DroneInterface(
            drone_id='drone0',
            use_sim_time=False,
            verbose=True
        )
        
        self.get_logger().info('Simple mission node initialized')
        
    def run_mission(self):
        """Execute a simple test mission"""
        try:
            # Wait for the drone to be ready
            self.get_logger().info('Waiting for drone to be ready...')
            self.drone.wait_for_takeoff_capability()
            self.get_logger().info('Drone is ready!')
            
            # Arm the drone
            self.get_logger().info('Arming drone...')
            if not self.drone.arm():
                self.get_logger().error('Failed to arm drone')
                return False
            self.get_logger().info('Drone armed successfully')
            
            # Takeoff
            self.get_logger().info('Taking off to 5 meters...')
            if not self.drone.takeoff(5.0, 2.0):  # height=5m, speed=2m/s
                self.get_logger().error('Takeoff failed')
                return False
            self.get_logger().info('Takeoff successful')
            
            # Wait a bit
            time.sleep(2)
            
            # Simple square pattern
            waypoints = [
                (5.0, 0.0, 5.0),   # East
                (5.0, 5.0, 5.0),   # North-East
                (0.0, 5.0, 5.0),   # North
                (0.0, 0.0, 5.0),   # Back to start
            ]
            
            for i, (x, y, z) in enumerate(waypoints):
                self.get_logger().info(f'Going to waypoint {i+1}: ({x}, {y}, {z})')
                success = self.drone.go_to_pose(
                    x, y, z, 
                    yaw_mode=YawMode.YAW_ANGLE,
                    yaw_angle=0.0,
                    speed=2.0
                )
                if not success:
                    self.get_logger().error(f'Failed to reach waypoint {i+1}')
                    break
                time.sleep(3)  # Wait at each waypoint
            
            # Land
            self.get_logger().info('Landing...')
            if not self.drone.land(2.0):  # speed=2m/s
                self.get_logger().error('Landing failed')
                return False
            self.get_logger().info('Landed successfully')
            
            # Disarm
            self.get_logger().info('Disarming...')
            if not self.drone.disarm():
                self.get_logger().error('Failed to disarm')
                return False
            self.get_logger().info('Mission completed successfully!')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Mission failed with exception: {str(e)}')
            return False

def main():
    rclpy.init()
    mission = SimpleMission()
    
    try:
        success = mission.run_mission()
        if success:
            mission.get_logger().info('Mission completed successfully!')
        else:
            mission.get_logger().error('Mission failed!')
    except KeyboardInterrupt:
        mission.get_logger().info('Mission interrupted by user')
    finally:
        mission.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
