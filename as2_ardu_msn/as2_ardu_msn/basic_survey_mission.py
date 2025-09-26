#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import yaml
import numpy as np
import time
from threading import Thread
import argparse
import sys
from rclpy.utilities import remove_ros_args

# Direct MAVROS imports
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, GlobalPositionTarget, PositionTarget
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

class BasicSurveyMission(Node):
    def __init__(self, config_file):
        super().__init__('survey_mission_node')
        
        # Load configuration
        self.get_logger().info(f"Loading config from: {config_file}")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['survey_parameters']
        
        # Initialize state tracking
        self.mavros_state = None
        self.current_position = None
        self.gps_status = None
        self.is_armed = False
        self.is_offboard = False
        
        # Flight parameters
        self.takeoff_altitude = 5.0  # meters
        self.flight_speed = 2.0  # m/s
        self.waypoint_tolerance = 1.0  # meters (increased for more reliable waypoint achievement)
        
        # QoS settings for MAVROS topics
        mavros_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscribe to MAVROS state
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_callback,
            mavros_qos
        )

        # Subscribe to local position
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_callback,
            mavros_qos
        )
        
        # Subscribe to GPS for pre-arm checks
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'mavros/global_position/global',
            self.gps_callback,
            mavros_qos
        )
        
        # Publishers for setpoints
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            mavros_qos
        )
        
        # Service clients
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, 'mavros/cmd/land')
        
        # Generate waypoints
        self.waypoints = self.generate_simple_pattern()
        self.current_waypoint_index = 0
        
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints for basic survey")
        self.get_logger().info("Direct MAVROS mission node initialized")
    
    def state_callback(self, msg):
        """Track MAVROS state changes"""
        if self.mavros_state is None or self.mavros_state.armed != msg.armed:
            self.get_logger().info(f"🔄 MAVROS state - Armed: {msg.armed}, Mode: {msg.mode}, Connected: {msg.connected}")
        
        self.mavros_state = msg
        self.is_armed = msg.armed
        self.is_offboard = (msg.mode == "OFFBOARD")
    
    def pose_callback(self, msg):
        """Track current position from MAVROS local position"""
        self.get_logger().info(f"📍 Pose callback received! Position: {msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f}")
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
    
    def gps_callback(self, msg):
        """Track GPS status for pre-arm checks"""
        self.gps_status = msg
        # GPS status: 
        # status.status: -1=NO_FIX, 0=FIX, 1=SBAS_FIX, 2=GBAS_FIX
        if msg.status.status >= 0:
            self.get_logger().info(f"🛰️ GPS Fix available: status={msg.status.status}, satellites={msg.status.service}")
        else:
            self.get_logger().debug(f"GPS No Fix: status={msg.status.status}")
    
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
        """Set GUIDED mode for ArduPilot"""
        if not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Set mode service not available")
            return False
        
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        
        try:
            future = self.set_mode_client.call_async(req)
            # Spin in a separate thread to avoid blocking
            timeout = 5.0
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done() and future.result().mode_sent:
                self.get_logger().info("GUIDED mode set successfully")
                time.sleep(2)  # Give ArduPilot time to process
                return True
            else:
                self.get_logger().error("Failed to set GUIDED mode")
                return False
        except Exception as e:
            self.get_logger().error(f"Error setting mode: {e}")
            return False
    
    def arm_drone(self):
        """Arm the drone using direct MAVROS commands"""
        self.get_logger().info("🔧 Arming via direct MAVROS...")
        
        if not self.arm_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("MAVROS arming service not available")
            return False
        
        try:
            request = CommandBool.Request()
            request.value = True
            
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info("✅ Drone armed successfully via MAVROS")
                return True
            else:
                result = future.result()
                if result:
                    self.get_logger().error(f"❌ Failed to arm: {result.result}")
                else:
                    self.get_logger().error("❌ Failed to arm: timeout")
                return False
        except Exception as e:
            self.get_logger().error(f"Error arming via MAVROS: {e}")
            return False
    
    def wait_for_armed_state(self, timeout_sec=15.0):
        """Wait for MAVROS to report armed state after arming command"""
        self.get_logger().info("🔍 Waiting for MAVROS armed state confirmation...")
        
        start_time = time.time()
        last_log = 0.0
        
        while (time.time() - start_time) < timeout_sec:
            # Check MAVROS state for armed confirmation
            if self.mavros_state and self.mavros_state.armed:
                self.get_logger().info("✅ MAVROS confirms armed state")
                return True
            
            # Periodic logging
            now = time.time()
            if now - last_log > 2.0:
                if self.mavros_state:
                    self.get_logger().info(f"MAVROS state: Mode={self.mavros_state.mode}, Armed={self.mavros_state.armed}")
                else:
                    self.get_logger().info("Waiting for MAVROS state...")
                last_log = now
            
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        self.get_logger().error("⏰ Timeout waiting for armed state confirmation")
        return False
    
    def takeoff_mavros(self, altitude):
        """Takeoff using direct MAVROS commands"""
        self.get_logger().info(f"Taking off to altitude: {altitude}m")
        
        # Wait for MAVROS services
        if not self.takeoff_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("MAVROS takeoff service not available")
            return False
        
        # Send takeoff command
        try:
            request = CommandTOL.Request()
            request.altitude = altitude
            
            future = self.takeoff_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info("Takeoff command sent successfully")
                
                # Wait for takeoff to complete by monitoring altitude
                start_time = time.time()
                timeout = 30.0
                target_altitude = altitude * 0.95  # 95% of target altitude
                
                while (time.time() - start_time) < timeout:
                    if self.current_position and self.current_position[2] >= target_altitude:
                        self.get_logger().info("✅ Takeoff completed successfully")
                        return True
                    
                    time.sleep(0.5)
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                self.get_logger().error("Takeoff timeout - altitude not reached")
                return False
            else:
                self.get_logger().error("Takeoff command failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error during takeoff: {e}")
            return False
    
    def goto_waypoint_mavros(self, waypoint, speed):
        """Navigate to waypoint using direct MAVROS setpoint publishing"""
        self.get_logger().info(f"Going to waypoint: {waypoint} at speed {speed}")
        
        # Create setpoint message
        setpoint = PoseStamped()
        setpoint.header.frame_id = "map"
        setpoint.pose.position.x = waypoint[0]
        setpoint.pose.position.y = waypoint[1]
        setpoint.pose.position.z = waypoint[2]
        setpoint.pose.orientation.w = 1.0
        
        start_time = time.time()
        timeout = 60.0
        last_publish_time = 0
        publish_rate = 0.05  # 20 Hz = 0.05 second interval
        
        while (time.time() - start_time) < timeout:
            current_time = time.time()
            
            # Publish at desired rate
            if (current_time - last_publish_time) >= publish_rate:
                setpoint.header.stamp = self.get_clock().now().to_msg()
                self.local_pos_pub.publish(setpoint)
                last_publish_time = current_time
            
            # Check if waypoint is reached
            if self.current_position:
                distance = np.sqrt(
                    (self.current_position[0] - waypoint[0])**2 +
                    (self.current_position[1] - waypoint[1])**2 +
                    (self.current_position[2] - waypoint[2])**2
                )
                
                if distance < self.waypoint_tolerance:
                    self.get_logger().info(f"✅ Reached waypoint: {waypoint}")
                    return True
            
            # Process callbacks and sleep
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.01)  # Small sleep to prevent CPU spinning
        
        self.get_logger().error("Waypoint navigation timeout")
        return False
    
    def land_mavros(self):
        """Land using direct MAVROS commands"""
        self.get_logger().info("Landing...")
        
        if not self.land_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("MAVROS land service not available")
            return False
        
        try:
            request = CommandTOL.Request()
            request.altitude = 0.0  # Land at ground level
            
            future = self.land_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info("Land command sent successfully")
                
                # Wait for landing to complete by monitoring altitude
                start_time = time.time()
                timeout = 30.0
                ground_threshold = 0.2  # Consider landed below 20cm
                
                while (time.time() - start_time) < timeout:
                    if self.current_position and self.current_position[2] <= ground_threshold:
                        self.get_logger().info("✅ Landing completed successfully")
                        return True
                    
                    time.sleep(0.5)
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                self.get_logger().error("Landing timeout")
                return False
            else:
                self.get_logger().error("Land command failed")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error during landing: {e}")
            return False
    
    def wait_for_mavros_ready(self, timeout_sec=10.0):
        """Wait for MAVROS connection and basic state availability"""
        self.get_logger().info("🔍 Waiting for MAVROS to be ready...")
        
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            # Check if we have MAVROS state
            if self.mavros_state and self.mavros_state.connected:
                self.get_logger().info("✅ MAVROS connection ready")
                return True
            
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        self.get_logger().warn(f"MAVROS readiness timeout after {timeout_sec}s")
        return False
    
    def wait_for_gps(self, timeout_sec=30.0):
        """Wait for GPS fix (optional for SITL)"""
        self.get_logger().info("🛰️ Checking GPS status...")
        
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.gps_status is not None:
                if self.gps_status.status.status >= 0:  # GPS fix available
                    self.get_logger().info(f"✅ GPS fix available (status: {self.gps_status.status.status})")
                    return True
                else:
                    # In SITL, we might not have real GPS, so we can proceed anyway
                    elapsed = time.time() - start_time
                    if elapsed > 10.0:  # After 10 seconds, proceed anyway for SITL
                        self.get_logger().warn("⚠️ No GPS fix but proceeding (SITL mode)")
                        return True
            
            time.sleep(0.5)
        
        # For SITL, we can proceed without GPS
        self.get_logger().warn("⚠️ GPS timeout, but proceeding anyway (SITL mode)")
        return True
    
    def wait_for_system_ready(self):
        """Wait for position to be available (platform state not required)"""
        self.get_logger().info("🟡 Waiting for system to be ready...")
        
        timeout = 30.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            position_ready = self.current_position is not None
            mavros_connected = self.mavros_state is not None and self.mavros_state.connected
            
            # Log status every 2 seconds to reduce spam
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0:
                self.get_logger().info(f"Position available: {position_ready}, MAVROS connected: {mavros_connected}")
            
            # Require position data and MAVROS connection
            if position_ready and mavros_connected:
                self.get_logger().info("✅ System ready! Position data available and MAVROS connected.")
                return True
            
            time.sleep(1)
        
        self.get_logger().error("System ready timeout - position data not available")
        return False
    
    def execute_mission(self):
        """Execute the survey mission using direct MAVROS commands"""
        try:
            self.get_logger().info("🚁 Starting Direct MAVROS Survey Mission")
            
            # Phase 1: Wait for MAVROS readiness
            self.get_logger().info("Phase 1: Waiting for MAVROS readiness")
            if not self.wait_for_mavros_ready(timeout_sec=15.0):
                self.get_logger().error("MAVROS not ready - aborting mission")
                return False
            
            # Phase 2: Set mode and arm
            self.get_logger().info("Phase 2: Setting GUIDED mode and arming")
            if not self.set_guided_mode():
                return False
            
            # Check GPS status before arming (helpful for diagnosis)
            self.wait_for_gps(timeout_sec=10.0)  # Short timeout for SITL
            
            if not self.arm_drone():
                return False
            
            # Wait for armed confirmation
            if not self.wait_for_armed_state(timeout_sec=10.0):
                self.get_logger().error("Failed to confirm armed state")
                return False
            
            # Phase 3: Takeoff
            self.get_logger().info("Phase 3: Taking off")
            takeoff_altitude = self.config.get('altitude', self.takeoff_altitude)
            if not self.takeoff_mavros(takeoff_altitude):
                return False
            
            # Phase 4: Execute survey pattern
            self.get_logger().info("Phase 4: Executing survey pattern")
            speed = self.config.get('flight_speed', self.flight_speed)
            
            for i, waypoint in enumerate(self.waypoints):
                self.get_logger().info(f"Waypoint {i+1}/{len(self.waypoints)}: {waypoint}")
                if not self.goto_waypoint_mavros(waypoint, speed):
                    self.get_logger().error(f"Failed to reach waypoint {i+1}")
                    return False
                
                # Small delay between waypoints
                time.sleep(1)
            
            # Phase 5: Land
            self.get_logger().info("Phase 5: Landing")
            if not self.land_mavros():
                return False
            
            self.get_logger().info("🎉 Direct MAVROS survey mission completed successfully!")
            return True
            
        except KeyboardInterrupt:
            self.get_logger().info("Mission interrupted by user")
            return False
        except Exception as e:
            self.get_logger().error(f"Mission failed with error: {e}")
            return False

def main():
    parser = argparse.ArgumentParser(description='Basic Survey Mission')
    parser.add_argument('--config', type=str, 
                       default='/home/dhandavishal/aerostack2_ws/src/as2_ardu_msn/config/mission_config.yaml',
                       help='Path to configuration file')
    
    # Remove ROS args to avoid conflicts and ignore unknown args
    args, unknown = parser.parse_known_args(remove_ros_args(sys.argv))
    
    rclpy.init()
    
    try:
        mission = BasicSurveyMission(args.config)
        
        # Execute mission directly without threading to avoid "wait set index too big" error
        success = mission.execute_mission()
        
        if success:
            print("✅ Mission completed successfully!")
        else:
            print("❌ Mission failed!")
            return 1
        
    except KeyboardInterrupt:
        print("\nMission interrupted by user")
        return 1
    except Exception as e:
        print(f"Mission failed: {e}")
        return 1
    finally:
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    exit(main())
