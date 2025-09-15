#!/usr/bin/env python3

import rclpy
import yaml
import numpy as np
import time
from as2_python_api.drone_interface import DroneInterface
import argparse
import sys
from rclpy.utilities import remove_ros_args

# The class no longer inherits from Node
class BasicSurveyMission:
    def __init__(self, config_file):
        # The logger is no longer available, so we use print()
        print(f"Loading config from: {config_file}")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['survey_parameters']
        
        # Initialize drone interface  
        self.drone = DroneInterface(
            drone_id='drone0',
            use_sim_time=False,
            verbose=True
        )
        
        self.waypoints = self.generate_simple_pattern()
        print(f"Generated {len(self.waypoints)} waypoints for basic survey")
    
    def generate_simple_pattern(self):
        waypoints = []
        width = self.config['area_width']
        length = self.config['area_length']
        altitude = self.config['altitude']
        spacing = self.config['line_spacing']
        origin_x = self.config['origin_offset']['x']
        origin_y = self.config['origin_offset']['y']
        num_lines = int(width / spacing) + 1
        
        for i in range(num_lines):
            y_pos = origin_y + i * spacing
            if i % 2 == 0:
                waypoints.append([origin_x, y_pos, altitude])
                waypoints.append([origin_x + length, y_pos, altitude])
            else:
                waypoints.append([origin_x + length, y_pos, altitude])
                waypoints.append([origin_x, y_pos, altitude])
        return waypoints
    
    def wait_for_drone_ready(self):
        print("Waiting for drone to be ready...")
        self.drone.wait_for_connection()  # Use the built-in wait function
        print("✓ Drone interface ready")
        
        print("Waiting for behaviors to stabilize...")
        time.sleep(5)
        
        print("✓ Drone ready")
        return True
    
    def execute_basic_mission(self):
        try:
            print("🚁 Starting Basic Survey Mission")
            if not self.wait_for_drone_ready():
                print("Drone not ready")
                return False
            
            print("Phase 1: Arming and takeoff")
            print("Arming drone...")
            self.drone.arm()
            print("✓ Drone armed")
            
            print("Setting offboard mode...")
            self.drone.offboard()
            print("✓ Offboard mode set")
            
            takeoff_height = self.config['altitude']
            print(f"Taking off to {takeoff_height}m...")
            self.drone.takeoff(height=takeoff_height, speed=self.config['transition_speed'])
            print("✓ Takeoff complete")
            time.sleep(3)
            
            print("Phase 2: Executing survey pattern")
            for i, waypoint in enumerate(self.waypoints):
                is_survey_line = (i % 2 == 1)
                speed = self.config['survey_speed'] if is_survey_line else self.config['transition_speed']
                
                print(f"[{i+1}/{len(self.waypoints)}] Going to waypoint: ({waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f}) at {speed:.1f} m/s")
                
                if is_survey_line:
                    self.drone.go_to.go_to_point_path_facing(waypoint, speed=speed)
                else:
                    self.drone.go_to.go_to_point(waypoint, speed=speed)
                
                if is_survey_line:
                    print("📸 Collecting survey data...")
                    time.sleep(1)
                else:
                    time.sleep(0.5)
            
            print("✓ Survey pattern complete")
            
            if self.config.get('rtl_on_completion', True):
                print("Phase 3: Returning to launch")
                home_position = [0.0, 0.0, self.config['altitude']]
                self.drone.go_to.go_to_point(home_position, speed=self.config['transition_speed'])
                print("✓ Returned to launch position")
            
            print("Phase 4: Landing")
            self.drone.land(speed=1.0)
            print("✓ Landing complete")
            
            print("Setting manual mode...")
            self.drone.manual()
            print("✓ Manual mode set")

            print("🎉 Basic survey mission completed successfully!")
            return True
            
        except Exception as e:
            print(f"MISSION ERROR: {e}")
            print("Executing emergency land...")
            try:
                self.drone.land(speed=2.0)
                time.sleep(3)
                self.drone.manual()
            except Exception as emergency_e:
                print(f"EMERGENCY LAND FAILED: {emergency_e}")
            return False

def main():
    rclpy.init()
    filtered_args = remove_ros_args(args=sys.argv[1:])
    
    parser = argparse.ArgumentParser(description='Basic Survey Mission for ArduPilot SITL')
    parser.add_argument('--config', type=str, default='config/mission_config.yaml', help='Path to mission configuration file')
    args = parser.parse_args(filtered_args)
    
    try:
        mission = BasicSurveyMission(args.config)
        
        if not mission.config.get('auto_start', False):
            print("\n🚁 Basic Survey Mission Ready")
            print(f"📍 Area: {mission.config['area_width']}m x {mission.config['area_length']}m")
            print(f"📏 Altitude: {mission.config['altitude']}m")
            print(f"📊 Waypoints: {len(mission.waypoints)}")
            print("\nPress Enter to start mission or Ctrl+C to abort...")
            input()
        
        success = mission.execute_basic_mission()
        mission.drone.shutdown()
        
        if success:
            print("Mission completed successfully")
            return 0
        else:
            print("Mission failed")
            return 1
            
    except KeyboardInterrupt:
        print("\nMission aborted by user")
        return 1
    except Exception as e:
        print(f"Mission failed with error: {e}")
        return 1
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    exit(main())