#!/usr/bin/env python3

import rclpy
import yaml
import time
from as2_python_api.drone_interface import DroneInterface
import argparse
import sys
from rclpy.utilities import remove_ros_args

class TakeoffMission:
    def __init__(self, config_file):
        print(f"Loading config from: {config_file}")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['takeoff_parameters']
        
        # Initialize drone interface  
        self.drone = DroneInterface(
            drone_id='drone0',
            use_sim_time=False,
            verbose=True
        )
        
        print(f"Takeoff mission configured for {self.config['altitude']}m altitude")
    
    def wait_for_drone_ready(self):
        """Wait for drone to be connected and ready"""
        print("Waiting for drone to be ready...")
        max_wait_time = 30  # seconds
        start_time = time.time()
        
        # Wait for the connection using the built-in method
        try:
            self.drone.wait_for_connection()
            print("âœ“ Drone interface connected")
        except Exception as e:
            print(f"âœ— Failed to connect: {e}")
            return False
        
        # Wait for behaviors to be ready
        print("Waiting for behaviors to stabilize...")
        time.sleep(5)  # Give time for all behaviors to initialize
        
        # Simple connectivity check - if we can check flying status, we're connected
        while time.time() - start_time < max_wait_time:
            try:
                # The is_flying() method should work if connected
                flying_status = self.drone.is_flying()
                if flying_status is not None:  # Returns True/False if connected
                    print(f"âœ“ Drone is ready (flying: {flying_status})")
                    return True
            except Exception:
                pass  # Still waiting for connection
            
            time.sleep(1)
        
        print("âœ— Timeout waiting for drone to be ready")
        return False
    
    def execute_takeoff_mission(self):
        """Execute the takeoff and landing mission"""
        
        try:
            print("\nðŸš Starting Takeoff Mission")
            print("=" * 50)
            
            # Wait for drone to be ready
            if not self.wait_for_drone_ready():
                print("MISSION ABORTED: Drone not ready")
                return False
            
            print("\nPhase 1: Arming")
            print("Arming drone...")
            self.drone.arm()
            print("âœ“ Drone armed")
            time.sleep(2)
            
            print("\nPhase 2: Set Offboard Mode")
            print("Setting offboard mode...")
            self.drone.offboard()
            print("âœ“ Offboard mode set")
            time.sleep(2)
            
            print(f"\nPhase 3: Takeoff")
            print(f"Taking off to {self.config['altitude']}m altitude...")
            
            # Takeoff to specified altitude
            success = self.drone.takeoff(
                height=self.config['altitude'], 
                speed=self.config['takeoff_speed']
            )
            
            if not success:
                print("âœ— Takeoff failed")
                return False
            
            print(f"âœ“ Successfully reached {self.config['altitude']}m altitude")
            
            # Hold position for specified time
            hold_time = self.config.get('hold_time', 10)
            if hold_time > 0:
                print(f"\nPhase 4: Holding position for {hold_time} seconds")
                time.sleep(hold_time)
                print("âœ“ Hold complete")
            
            # Optional: Move to a different position if configured
            if 'waypoint' in self.config and self.config['waypoint']['enabled']:
                print("\nPhase 5: Moving to waypoint")
                waypoint = [
                    self.config['waypoint']['x'],
                    self.config['waypoint']['y'],
                    self.config['altitude']
                ]
                self.drone.go_to.go_to_point(waypoint, speed=self.config['flight_speed'])
                print(f"âœ“ Reached waypoint: {waypoint}")
                
                # Hold at waypoint
                waypoint_hold = self.config['waypoint'].get('hold_time', 5)
                if waypoint_hold > 0:
                    print(f"Holding at waypoint for {waypoint_hold} seconds")
                    time.sleep(waypoint_hold)
            
            print("\nPhase 6: Landing")
            print("Initiating landing sequence...")
            
            # Land the drone
            self.drone.land(speed=self.config['landing_speed'])
            print("âœ“ Landing complete")
            time.sleep(2)
            
            # Disarm
            print("\nPhase 7: Disarming")
            self.drone.disarm()
            print("âœ“ Drone disarmed")
            
            # Set to manual mode
            print("Setting manual mode...")
            self.drone.manual()
            print("âœ“ Manual mode set")

            print("\nðŸŽ‰ Takeoff mission completed successfully!")
            return True
            
        except Exception as e:
            print(f"\nMISSION ERROR: {e}")
            print("Executing emergency procedures...")
            try:
                # Try to land
                print("Emergency landing...")
                self.drone.land(speed=2.0)
                time.sleep(3)
                
                # Disarm
                self.drone.disarm()
                
                # Set manual mode
                self.drone.manual()
                print("âœ“ Emergency procedures complete")
            except Exception as emergency_e:
                print(f"EMERGENCY PROCEDURES FAILED: {emergency_e}")
            return False

def main():
    rclpy.init()
    filtered_args = remove_ros_args(args=sys.argv[1:])
    
    parser = argparse.ArgumentParser(description='Simple Takeoff Mission for ArduPilot SITL')
    parser.add_argument('--config', type=str, default='config/takeoff_config.yaml', help='Path to takeoff configuration file')
    args = parser.parse_args(filtered_args)
    
    try:
        mission = TakeoffMission(args.config)
        
        if not mission.config.get('auto_start', False):
            # Wait for system to stabilize
            print("\nWaiting for system to stabilize...")
            time.sleep(30)  # Give more time for all nodes to initialize
            
            print("\nðŸš Takeoff Mission Ready")
            print(f"ðŸ“ Target Altitude: {mission.config['altitude']}m")
            print(f"â¬†ï¸  Takeoff Speed: {mission.config['takeoff_speed']}m/s")
            print(f"â¬‡ï¸  Landing Speed: {mission.config['landing_speed']}m/s")
            if 'waypoint' in mission.config and mission.config['waypoint']['enabled']:
                print(f"ðŸ“ Waypoint: ({mission.config['waypoint']['x']}, {mission.config['waypoint']['y']})")
            print("\nStarting Mission...")

        success = mission.execute_takeoff_mission()
        mission.drone.shutdown()
        
        if success:
            print("\nMission completed successfully")
            return 0
        else:
            print("\nMission failed")
            return 1
            
    except KeyboardInterrupt:
        print("\nMission aborted by user")
        try:
            mission.drone.land(speed=2.0)
            mission.drone.disarm()
            mission.drone.manual()
            mission.drone.shutdown()
        except:
            pass
        return 1
    except Exception as e:
        print(f"Mission failed with error: {e}")
        return 1
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    exit(main())