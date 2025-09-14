#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
import time
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.msg import YawMode
import argparse

class BasicSurveyMission(Node):
    def __init__(self, config_file):
        super().__init__('basic_survey_mission')
        
        # Load configuration
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['survey_parameters']
        
        # Initialize drone interface  
        self.drone = DroneInterface(
            drone_id='drone0',
            use_sim_time=False,  # Match launch file setting
            verbose=True
        )
        
        # Generate simple waypoints for survey
        self.waypoints = self.generate_simple_pattern()
        
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints for basic survey")
    
    def generate_simple_pattern(self):
        """Generate a simple rectangular survey pattern"""
        waypoints = []
        
        # Get parameters
        width = self.config['area_width']
        length = self.config['area_length']
        altitude = self.config['altitude']
        spacing = self.config['line_spacing']
        origin_x = self.config['origin_offset']['x']
        origin_y = self.config['origin_offset']['y']
        
        # Calculate number of lines
        num_lines = int(width / spacing) + 1
        
        # Generate lawnmower pattern
        for i in range(num_lines):
            y_pos = origin_y + i * spacing
            
            if i % 2 == 0:  # Even lines: left to right
                # Start point
                waypoints.append([origin_x, y_pos, altitude])
                # End point
                waypoints.append([origin_x + length, y_pos, altitude])
            else:  # Odd lines: right to left
                # Start point
                waypoints.append([origin_x + length, y_pos, altitude])
                # End point
                waypoints.append([origin_x, y_pos, altitude])
        
        return waypoints
    
    def wait_for_drone_ready(self):
        """Wait for drone systems to be ready"""
        self.get_logger().info("Waiting for drone to be ready...")
        
        # Simple wait approach with better checking
        max_wait_time = 30  # seconds
        start_time = time.time()
        
        while time.time() - start_time < max_wait_time:
            try:
                # Check if the drone interface is properly initialized
                # For as2_python_api, we can check if behaviors are available
                if hasattr(self.drone, 'go_to') and hasattr(self.drone, 'takeoff'):
                    self.get_logger().info("✓ Drone interface ready")
                    break
                
                # Wait a bit before retrying
                time.sleep(1)
                self.get_logger().info("Waiting for drone interface...")
                
            except Exception as e:
                self.get_logger().info(f"Waiting for drone interface: {e}")
                time.sleep(1)
        else:
            self.get_logger().warn("Timeout waiting for drone, proceeding anyway...")
        
        # Additional wait for behaviors to be ready
        self.get_logger().info("Waiting for behaviors to stabilize...")
        time.sleep(5)
        
        self.get_logger().info("✓ Drone ready")
        return True
    
    def execute_basic_mission(self):
        """Execute the basic survey mission"""
        try:
            self.get_logger().info("🚁 Starting Basic Survey Mission")
            
            # Wait for drone to be ready
            if not self.wait_for_drone_ready():
                self.get_logger().error("Drone not ready")
                return False
            
            # Phase 1: Arm and takeoff
            self.get_logger().info("Phase 1: Arming and takeoff")
            
            # Arm the drone
            self.get_logger().info("Arming drone...")
            if not self.drone.arm():
                self.get_logger().error("Failed to arm drone")
                return False
            self.get_logger().info("✓ Drone armed")
            
            # Set offboard mode
            self.get_logger().info("Setting offboard mode...")
            if not self.drone.offboard():
                self.get_logger().error("Failed to set offboard mode")
                return False
            self.get_logger().info("✓ Offboard mode set")
            
            # Takeoff
            takeoff_height = self.config['altitude']
            self.get_logger().info(f"Taking off to {takeoff_height}m...")
            if not self.drone.takeoff(
                height=takeoff_height,
                speed=self.config['transition_speed']
            ):
                self.get_logger().error("Takeoff failed")
                return False
            
            self.get_logger().info("✓ Takeoff complete")
            time.sleep(3)  # Stabilize
            
            # Phase 2: Execute survey pattern
            self.get_logger().info("Phase 2: Executing survey pattern")
            
            for i, waypoint in enumerate(self.waypoints):
                # Determine if this is a survey line or transition
                is_survey_line = (i % 2 == 1)  # Odd indices are end of survey lines
                speed = self.config['survey_speed'] if is_survey_line else self.config['transition_speed']
                
                self.get_logger().info(
                    f"[{i+1}/{len(self.waypoints)}] Going to waypoint: "
                    f"({waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f}) "
                    f"at {speed:.1f} m/s"
                )
                
                # Move to waypoint using go_to behavior with path facing for better coverage
                if is_survey_line:
                    success = self.drone.go_to.go_to_point_path_facing(
                        waypoint,
                        speed=speed
                    )
                else:
                    success = self.drone.go_to.go_to_point(
                        waypoint,
                        speed=speed
                    )
                
                if not success:
                    self.get_logger().error(f"Failed to reach waypoint {i+1}")
                    return False
                
                # Small pause for survey data collection simulation
                if is_survey_line:
                    self.get_logger().info("📸 Collecting survey data...")
                    time.sleep(1)
                else:
                    # Small pause for transitions
                    time.sleep(0.5)
            
            self.get_logger().info("✓ Survey pattern complete")
            
            # Phase 3: Return to launch (if configured)
            if self.config.get('rtl_on_completion', True):
                self.get_logger().info("Phase 3: Returning to launch")
                
                # Return to home position
                home_position = [0.0, 0.0, self.config['altitude']]
                success = self.drone.go_to.go_to_point(
                    home_position,
                    speed=self.config['transition_speed']
                )
                
                if not success:
                    self.get_logger().error("Failed to return to launch")
                    return False
                
                self.get_logger().info("✓ Returned to launch position")
            
            # Phase 4: Landing
            self.get_logger().info("Phase 4: Landing")
            if not self.drone.land(speed=1.0):
                self.get_logger().error("Landing failed")
                return False
            
            self.get_logger().info("✓ Landing complete")
            
            # Set manual mode
            self.get_logger().info("Setting manual mode...")
            if not self.drone.manual():
                self.get_logger().error("Failed to set manual mode")
                # Continue anyway
            else:
                self.get_logger().info("✓ Manual mode set")
            
            # Disarm (optional - manual mode usually handles this)
            # self.get_logger().info("Disarming drone...")
            # self.drone.disarm()
            # self.get_logger().info("✓ Drone disarmed")
            
            self.get_logger().info("🎉 Basic survey mission completed successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Mission error: {e}")
            # Emergency landing
            self.get_logger().info("Executing emergency landing...")
            try:
                self.drone.land(speed=2.0)
                time.sleep(3)  # Wait for landing
                self.drone.manual()  # Set manual mode
            except Exception as emergency_e:
                self.get_logger().error(f"Emergency landing failed: {emergency_e}")
            return False

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Basic Survey Mission for ArduPilot SITL')
    parser.add_argument('--config', type=str, 
                       default='config/mission_config.yaml',
                       help='Path to mission configuration file')
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create mission node
        mission = BasicSurveyMission(args.config)
        
        # Check if auto-start is enabled
        if not mission.config.get('auto_start', False):
            print("\n🚁 Basic Survey Mission Ready")
            print(f"📍 Area: {mission.config['area_width']}m x {mission.config['area_length']}m")
            print(f"📏 Altitude: {mission.config['altitude']}m")
            print(f"📊 Waypoints: {len(mission.waypoints)}")
            print("\nPress Enter to start mission or Ctrl+C to abort...")
            input()
        
        # Execute mission
        success = mission.execute_basic_mission()
        
        # Cleanup
        mission.drone.shutdown()
        
        if success:
            mission.get_logger().info("Mission completed successfully")
            return 0
        else:
            mission.get_logger().error("Mission failed")
            return 1
            
    except KeyboardInterrupt:
        print("\nMission aborted by user")
        return 1
    except Exception as e:
        print(f"Mission failed with error: {e}")
        return 1
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    exit(main())
