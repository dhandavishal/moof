#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
from time import sleep
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import YawMode
import argparse

class ArduPilotSurveyMission(Node):
    def __init__(self, config_file):
        super().__init__('ardupilot_survey_mission')
        
        # Load configuration
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['survey_parameters']
        
        # Initialize drone interface
        self.drone = DroneInterface(
            drone_id='drone0',
            use_sim_time=True,
            verbose=True
        )
        
        # Generate survey waypoints
        self.waypoints = self.generate_lawnmower_pattern()
        
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints for survey")
        
    def generate_lawnmower_pattern(self):
        """Generate lawnmower pattern waypoints"""
        waypoints = []
        
        # Get parameters
        width = self.config['area_width']
        length = self.config['area_length']
        altitude = self.config['altitude']
        spacing = self.config['line_spacing']
        overlap = self.config['overlap_percentage']
        origin_x = self.config['origin_offset']['x']
        origin_y = self.config['origin_offset']['y']
        
        # Calculate effective spacing
        effective_spacing = spacing * (1 - overlap / 100)
        
        # Calculate number of passes
        num_passes = int(np.ceil(width / effective_spacing))
        
        # Generate waypoints
        for i in range(num_passes):
            x = origin_x - width/2 + i * effective_spacing
            
            if i % 2 == 0:
                # Forward pass
                waypoints.append([x, origin_y - length/2, altitude])
                waypoints.append([x, origin_y + length/2, altitude])
            else:
                # Backward pass
                waypoints.append([x, origin_y + length/2, altitude])
                waypoints.append([x, origin_y - length/2, altitude])
        
        return waypoints
    
    def wait_for_ardupilot_ready(self):
        """Wait for ArduPilot to be ready"""
        self.get_logger().info("Waiting for ArduPilot SITL to be ready...")
        
        # Wait for MAVROS connection
        while not self.drone.info.connected:
            self.get_logger().info("Waiting for MAVROS connection...")
            sleep(1)
        
        self.get_logger().info("✓ Connected to ArduPilot SITL")
        
        # Wait for GPS fix (important for ArduPilot)
        # In real implementation, check GPS status topic
        sleep(5)
        
        return True
    
    def execute_mission(self):
        """Execute the survey mission"""
        
        # Wait for system ready
        if not self.wait_for_ardupilot_ready():
            return False
        
        # Safety check
        if not self.config['auto_start']:
            response = input("\n🚁 Ready to start mission. Continue? (y/n): ")
            if response.lower() != 'y':
                self.get_logger().info("Mission cancelled by user")
                return False
        
        try:
            # Phase 1: Arm and takeoff
            self.get_logger().info("Phase 1: Arming and takeoff")
            
            # Arm the drone
            self.get_logger().info("Arming...")
            if not self.drone.arm():
                self.get_logger().error("Failed to arm")
                return False
            
            # Set offboard/guided mode
            self.get_logger().info("Setting offboard mode...")
            if not self.drone.offboard():
                self.get_logger().error("Failed to set offboard mode")
                return False
            
            # Takeoff
            self.get_logger().info(f"Taking off to {self.config['altitude']}m...")
            if not self.drone.takeoff(
                height=self.config['altitude'],
                speed=self.config['transition_speed']
            ):
                self.get_logger().error("Takeoff failed")
                return False
            
            self.get_logger().info("✓ Takeoff complete")
            sleep(2)  # Stabilize
            
            # Phase 2: Execute survey
            self.get_logger().info("Phase 2: Starting survey pattern")
            
            for i, waypoint in enumerate(self.waypoints):
                # Determine speed
                is_transition = (i > 0 and i % 2 == 0)
                speed = (self.config['transition_speed'] if is_transition 
                        else self.config['survey_speed'])
                
                self.get_logger().info(
                    f"[{i+1}/{len(self.waypoints)}] Flying to "
                    f"({waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f}) "
                    f"at {speed:.1f} m/s"
                )
                
                # Send waypoint command
                success = self.drone.go_to.go_to_point_path_facing(
                    waypoint,
                    speed=speed
                )
                
                if not success:
                    self.get_logger().error(f"Failed to reach waypoint {i+1}")
                    return False
                
                # Optional: Add hover time for photo capture
                if not is_transition:
                    sleep(0.5)
            
            self.get_logger().info("✓ Survey pattern complete")
            
            # Phase 3: Return to launch
            if self.config['rtl_on_completion']:
                self.get_logger().info("Phase 3: Returning to launch")
                
                # Return to home position
                home = [0.0, 0.0, self.config['altitude']]
                self.drone.go_to.go_to_point(
                    home,
                    speed=self.config['transition_speed']
                )
            
            # Phase 4: Landing
            self.get_logger().info("Phase 4: Landing")
            if not self.drone.land(speed=1.0):
                self.get_logger().error("Landing failed")
                return False
            
            self.get_logger().info("✓ Mission complete!")
            
            # Disarm
            self.drone.disarm()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Mission error: {e}")
            # Emergency landing
            self.get_logger().info("Executing emergency landing...")
            self.drone.land(speed=2.0)
            return False
        
    def shutdown(self):
        """Clean shutdown"""
        self.drone.shutdown()

def main():
    parser = argparse.ArgumentParser(description='ArduPilot Survey Mission')
    parser.add_argument('--config', type=str, 
                       default='config/mission_config.yaml',
                       help='Path to mission configuration file')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    mission = ArduPilotSurveyMission(args.config)
    success = mission.execute_mission()
    
    mission.shutdown()
    rclpy.shutdown()
    
    return 0 if success else 1

if __name__ == '__main__':
    exit(main())