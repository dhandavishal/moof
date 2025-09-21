#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import numpy as np
import time
from as2_python_api.drone_interface import DroneInterface
from mavros_msgs.srv import SetMode
import argparse
import sys
from rclpy.utilities import remove_ros_args

class BasicSurveyMission(Node):
    def __init__(self, config_file):
        super().__init__('survey_mission_node')
        
        # Load configuration
        self.get_logger().info(f"Loading config from: {config_file}")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['survey_parameters']
        
        # Initialize drone interface  
        self.drone = DroneInterface(
            drone_id='drone0',
            use_sim_time=False,
            verbose=True
        )
        
        # Create service client for mode setting
        self.set_mode_client = self.create_client(
            SetMode, 
            '/drone0/mavros/set_mode'
        )
        
        self.waypoints = self.generate_simple_pattern()
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints for basic survey")
    
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
    
    def set_guided_mode(self):
        """Set GUIDED mode for ArduPilot instead of OFFBOARD"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        # Create request for GUIDED mode
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        
        # Call the service
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info("GUIDED mode set")
            return True
        else:
            self.get_logger().error("Failed to set GUIDED mode")
            return False
    
    def set_stabilize_mode(self):
        """Set STABILIZE mode for ArduPilot"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("Set mode service not available for STABILIZE")
            return False
        
        req = SetMode.Request()
        req.custom_mode = 'STABILIZE'
        
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info("STABILIZE mode set")
            return True
        else:
            self.get_logger().warn("Failed to set STABILIZE mode")
            return False
    
    def wait_for_drone_ready(self):
        self.get_logger().info("Waiting for drone to be ready...")
        
        # Wait for the drone interface to be ready
        timeout = 30.0  # 30 seconds timeout
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            try:
                # Check if we can get the drone's position (indicates connection)
                position = self.drone.position
                self.get_logger().info("Drone interface ready")
                break
            except Exception:
                time.sleep(1)
        else:
            self.get_logger().error("Timeout waiting for drone connection")
            return False
        
        self.get_logger().info("Waiting for behaviors to stabilize...")
        time.sleep(5)
        
        self.get_logger().info("Drone ready")
        return True
    
    def execute_basic_mission(self):
        try:
            self.get_logger().info("🚁 Starting Basic Survey Mission")
            if not self.wait_for_drone_ready():
                self.get_logger().error("Drone not ready")
                return False
            
            self.get_logger().info("Phase 1: Arming and takeoff")
            self.get_logger().info("Arming drone...")
            success = self.drone.arm()
            if not success:
                self.get_logger().error("Failed to arm drone")
                return False
            self.get_logger().info("Drone armed")
            
            # Use GUIDED mode instead of OFFBOARD for ArduPilot
            self.get_logger().info("Setting GUIDED mode...")
            success = self.set_guided_mode()
            if not success:
                self.get_logger().error("Failed to set GUIDED mode")
                return False
            
            takeoff_height = self.config['altitude']
            self.get_logger().info(f"Taking off to {takeoff_height}m...")
            success = self.drone.takeoff(height=takeoff_height, speed=self.config['transition_speed'])
            if not success:
                self.get_logger().error("Takeoff failed")
                return False
            self.get_logger().info("Takeoff complete")
            time.sleep(3)
            
            self.get_logger().info("Phase 2: Executing survey pattern")
            for i, waypoint in enumerate(self.waypoints):
                is_survey_line = (i % 2 == 1)
                speed = self.config['survey_speed'] if is_survey_line else self.config['transition_speed']
                
                self.get_logger().info(f"[{i+1}/{len(self.waypoints)}] Going to waypoint: ({waypoint[0]:.1f}, {waypoint[1]:.1f}, {waypoint[2]:.1f}) at {speed:.1f} m/s")
                
                if is_survey_line:
                    self.drone.go_to.go_to_point_path_facing(waypoint, speed=speed)
                else:
                    self.drone.go_to.go_to_point(waypoint, speed=speed)
                
                if is_survey_line:
                    self.get_logger().info("📸 Collecting survey data...")
                    time.sleep(1)
                else:
                    time.sleep(0.5)
            
            self.get_logger().info("Survey pattern complete")
            
            if self.config.get('rtl_on_completion', True):
                self.get_logger().info("Phase 3: Returning to launch")
                home_position = [0.0, 0.0, self.config['altitude']]
                self.drone.go_to.go_to_point(home_position, speed=self.config['transition_speed'])
                self.get_logger().info("Returned to launch position")
            
            self.get_logger().info("Phase 4: Landing")
            success = self.drone.land(speed=1.0)
            if not success:
                self.get_logger().error("Landing failed")
                return False
            self.get_logger().info("Landing complete")
            
            self.get_logger().info("Setting STABILIZE mode...")
            self.set_stabilize_mode()

            self.get_logger().info("🎉 Basic survey mission completed successfully!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"MISSION ERROR: {e}")
            self.get_logger().info("Executing emergency land...")
            try:
                self.drone.land(speed=2.0)
                time.sleep(3)
                self.set_stabilize_mode()
            except Exception as emergency_e:
                self.get_logger().error(f"EMERGENCY LAND FAILED: {emergency_e}")
            return False

def main():
    rclpy.init()
    filtered_args = remove_ros_args(args=sys.argv[1:])
    
    parser = argparse.ArgumentParser(description='Basic Survey Mission for ArduPilot SITL')
    parser.add_argument('--config', type=str, default='config/mission_config.yaml', help='Path to mission configuration file')
    args = parser.parse_args(filtered_args)
    
    try:
        # Create the mission node
        mission = BasicSurveyMission(args.config)
        
        if not mission.config.get('auto_start', False):
            mission.get_logger().info("\n🚁 Basic Survey Mission Ready")
            mission.get_logger().info(f"📏 Area: {mission.config['area_width']}m x {mission.config['area_length']}m")
            mission.get_logger().info(f"📐 Altitude: {mission.config['altitude']}m")
            mission.get_logger().info(f"📊 Waypoints: {len(mission.waypoints)}")
            mission.get_logger().info("\nMission starting in 30 seconds...")
            time.sleep(30)
        
        # Execute mission in a separate thread to allow spinning
        from threading import Thread
        mission_thread = Thread(target=mission.execute_basic_mission)
        mission_thread.start()
        
        # Spin the node to handle callbacks
        rclpy.spin(mission)
        
        # Wait for mission to complete
        mission_thread.join()
        
        # Shutdown
        mission.drone.shutdown()
        mission.destroy_node()
        
        return 0
            
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