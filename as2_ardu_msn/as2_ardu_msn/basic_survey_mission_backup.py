#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.action import ActionClient
import yaml
import numpy as np
import time
from threading import Thread
import argparse
import sys
from rclpy.utilities import remove_ros_args
import tf2_ros
from rclpy.time import Time
import rclpy.duration

# Aerostack2 imports
from as2_python_api.drone_interface import DroneInterface
from as2_msgs.action import Takeoff, GoToWaypoint, Land
from as2_msgs.msg import PlatformInfo, YawMode
from geometry_msgs.msg import TwistStamped, PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from sensor_msgs.msg import NavSatFix

class BasicSurveyMission(Node):
    def __init__(self, config_file):
        super().__init__('survey_mission_node')
        
        # Set TF timeout parameters as per documentation
        self.declare_parameter('tf_timeout_threshold', 0.15)  # Increased from default 0.05
        
        # Load configuration
        self.get_logger().info(f"Loading config from: {config_file}")
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['survey_parameters']
        
        # Initialize platform state tracking
        self.platform_state = None
        self.platform_info = None  # full PlatformInfo message cache
        self.current_position = None
        self.gps_status = None
        self.is_armed = False
        self.is_flying = False
        
        # QoS settings matching typical Aerostack2 publishers:
        #  - platform/info: reliable + volatile
        #  - pose topics (sensor/self localization): best effort + volatile
        platform_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Subscribe to platform info to track state
        self.platform_info_sub = self.create_subscription(
            PlatformInfo,
            'platform/info',
            self.platform_info_callback,
            platform_qos
        )

        # Primary pose source
        self.pose_sub = self.create_subscription(
            PoseStamped,
            'sensor_measurements/pose',
            self.pose_callback,
            pose_qos
        )

        # Fallback pose from self_localization (some Aerostack2 setups publish pose here)
        self.fallback_pose_sub = self.create_subscription(
            PoseStamped,
            'self_localization/pose',
            self.pose_callback,
            pose_qos
        )
        
        # Subscribe to GPS for pre-arm checks (QoS matching MAVROS BEST_EFFORT)
        gps_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Match MAVROS
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.gps_sub = self.create_subscription(
            NavSatFix,
            'mavros/global_position/global',
            self.gps_callback,
            gps_qos
        )
        
        # Create MAVROS service clients
        self.set_mode_client = self.create_client(
            SetMode, 
            'mavros/set_mode'  # Relative path - namespace is handled by node launch
        )
        
        self.arm_client = self.create_client(
            CommandBool,
            'mavros/cmd/arming'  # Relative path - namespace is handled by node launch
        )
        
        # Create Aerostack2 action clients
        self.takeoff_client = ActionClient(self, Takeoff, 'TakeoffBehavior')  # Relative path - namespace is handled by node launch
        self.goto_client = ActionClient(self, GoToWaypoint, 'GoToBehavior')  # Relative path - namespace is handled by node launch
        self.land_client = ActionClient(self, Land, 'LandBehavior')  # Relative path - namespace is handled by node launch
        
        # Initialize action clients for drone operations
        self.drone_interface = DroneInterface(
            'drone0', 
            verbose=True,
            use_sim_time=True,
            spin_rate=20.0
        )
        
        # Initialize TF buffer and listener for readiness checks
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.waypoints = self.generate_simple_pattern()
        self.get_logger().info(f"Generated {len(self.waypoints)} waypoints for basic survey")
    
    def platform_info_callback(self, msg):
        """Track platform state changes"""
        self.get_logger().info(f"🔄 Platform callback received! State: {msg.status.state}, Armed: {msg.armed}")
        # cache entire message for later detailed checks
        self.platform_info = msg
        if self.platform_state != msg.status.state:
            self.get_logger().info(f"Platform state changed: {msg.status.state}")
            self.platform_state = msg.status.state
        # Always update these (even if same state) to keep them current
        self.is_armed = msg.armed
        self.is_flying = (msg.status.state in [3, 4, 5])  # TAKING_OFF, FLYING, LANDING
    
    def pose_callback(self, msg):
        """Track current position"""
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
    
    def arm_drone_aerostack2(self):
        """Arm the drone using Aerostack2 DroneInterface (proper FSM transition)"""
        self.get_logger().info("🔧 Arming via Aerostack2 DroneInterface...")
        
        try:
            # Use DroneInterface to arm (this updates the platform FSM properly)
            success = self.drone_interface.arm()
            if success:
                self.get_logger().info("✅ Drone armed successfully via Aerostack2")
                return True
            else:
                self.get_logger().error("❌ Failed to arm via Aerostack2")
                return False
        except Exception as e:
            self.get_logger().error(f"Error arming via Aerostack2: {e}")
            return False
    
    def wait_for_armed_state(self, timeout_sec=15.0):
        """Wait for platform to report armed state after arming command.

        Uses cached PlatformInfo. Platform considered armed when:
          - platform_info.armed is True AND
          - platform_status.state != 0 (not DISARMED)
        """
        self.get_logger().info("🔍 Waiting for platform armed state confirmation...")

        start_time = time.time()
        last_log = 0.0
        while (time.time() - start_time) < timeout_sec:
            if self.platform_info is not None:
                armed_state = self.platform_info.armed
                state = self.platform_info.status.state

                now = time.time()
                if now - last_log > 1.5:  # rate-limit logs
                    self.get_logger().info(f"Platform state: {state}, Armed: {armed_state}")
                    last_log = now

                if armed_state and state != 0:
                    self.get_logger().info("✅ Platform confirmed armed state")
                    return True

            time.sleep(0.25)

        self.get_logger().error("❌ Platform did not confirm armed state within timeout")
        return False
    
    def takeoff_aerostack2(self, altitude):
        """Takeoff using Aerostack2 action with proper feedback handling"""
        self.get_logger().info(f"Taking off to altitude: {altitude}m")
        
        if not self.takeoff_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Takeoff action server not available")
            return False
        
        goal = Takeoff.Goal()
        goal.takeoff_height = altitude
        goal.takeoff_speed = 1.0
        
        try:
            # Send goal and wait for result with proper feedback handling
            future = self.takeoff_client.send_goal_async(
                goal, 
                feedback_callback=self.takeoff_feedback_callback
            )
            
            # Wait for goal to be accepted
            timeout = 5.0
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error("Takeoff goal submission timeout")
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Takeoff goal rejected")
                return False
            
            self.get_logger().info("Takeoff goal accepted, waiting for completion...")
            
            # Wait for takeoff to complete
            result_future = goal_handle.get_result_async()
            timeout = 30.0  # Takeoff can take time
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
            if result_future.done():
                result = result_future.result()
                if result.result.success:
                    self.get_logger().info("✅ Takeoff completed successfully")
                    self.is_flying = True
                    return True
                else:
                    self.get_logger().error(f"Takeoff failed: {result.result.message}")
                    return False
            else:
                self.get_logger().error("Takeoff timeout")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error during takeoff: {e}")
            return False
    
    def takeoff_feedback_callback(self, feedback_msg):
        """Handle takeoff feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Takeoff progress: {feedback}")
    
    def goto_waypoint(self, waypoint, speed):
        """Navigate to waypoint using Aerostack2 GoToWaypoint action"""
        self.get_logger().info(f"Going to waypoint: {waypoint} at speed {speed}")
        
        if not self.goto_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("GoToWaypoint action server not available")
            return False
        
        goal = GoToWaypoint.Goal()
        goal.target_pose.header.frame_id = "earth"
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.position.z = waypoint[2]
        goal.target_pose.pose.orientation.w = 1.0
        goal.max_speed = speed
        
        # Set yaw mode
        goal.yaw.mode = YawMode.KEEP_YAW
        
        try:
            future = self.goto_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            timeout = 5.0
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error("GoToWaypoint goal submission timeout")
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("GoToWaypoint goal rejected")
                return False
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            timeout = 60.0  # Navigation can take time
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
            if result_future.done():
                result = result_future.result()
                if result.result.success:
                    self.get_logger().info(f"✅ Reached waypoint: {waypoint}")
                    return True
                else:
                    self.get_logger().error(f"Failed to reach waypoint: {result.result.message}")
                    return False
            else:
                self.get_logger().error("GoToWaypoint timeout")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error navigating to waypoint: {e}")
            return False
    
    def land_aerostack2(self):
        """Land using Aerostack2 action"""
        self.get_logger().info("Landing...")
        
        if not self.land_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Land action server not available")
            return False
        
        goal = Land.Goal()
        goal.land_speed = 0.5
        
        try:
            future = self.land_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            timeout = 5.0
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error("Land goal submission timeout")
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Land goal rejected")
                return False
            
            # Wait for landing completion
            result_future = goal_handle.get_result_async()
            timeout = 30.0
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
            if result_future.done():
                result = result_future.result()
                if result.result.success:
                    self.get_logger().info("✅ Landing completed successfully")
                    self.is_flying = False
                    return True
                else:
                    self.get_logger().error(f"Landing failed: {result.result.message}")
                    return False
            else:
                self.get_logger().error("Landing timeout")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error during landing: {e}")
            return False
    
    def wait_for_tf_ready(self, timeout_sec=10.0):
        """Wait for TF transforms to be available using ROS 2 TF2 patterns"""
        self.get_logger().info("🔍 Waiting for TF transforms to be ready...")
        
        source_frame = "earth"
        target_frame = "drone0/base_link"
        
        start_time = time.time()
        while (time.time() - start_time) < timeout_sec:
            try:
                # Check if transform is available
                if self.tf_buffer.can_transform(
                    target_frame, 
                    source_frame, 
                    rclpy.time.Time(),  # Latest available time
                    timeout=rclpy.duration.Duration(seconds=0.1)
                ):
                    # Transform is available, get it to verify it's recent
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            target_frame,
                            source_frame,
                            rclpy.time.Time(),  # Latest available
                            timeout=rclpy.duration.Duration(seconds=0.1)
                        )
                        self.get_logger().info(f"✅ TF transform ready: {source_frame} -> {target_frame}")
                        return True
                    except tf2_ros.TransformException as e:
                        self.get_logger().debug(f"Transform available but lookup failed: {e}")
                
            except tf2_ros.TransformException as e:
                self.get_logger().debug(f"TF not ready: {e}")
            
            # Spin once to process TF updates
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)  # Small delay between checks
        
        self.get_logger().warn(f"TF readiness timeout after {timeout_sec}s")
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
            
            # Log status every 2 seconds to reduce spam
            elapsed = time.time() - start_time
            if int(elapsed) % 2 == 0:
                self.get_logger().info(f"Position available: {position_ready}, Platform state: {self.platform_state}")
            
            # Only require position data - platform state is not critical for mission execution
            if position_ready:
                self.get_logger().info("✅ System ready! Position data available.")
                if self.platform_state is not None:
                    self.get_logger().info(f"🔄 Platform state: {self.platform_state}")
                else:
                    self.get_logger().warn("⚠️  Platform state not available, but proceeding with mission (pose data sufficient)")
                return True
            
            time.sleep(1)
        
        self.get_logger().error("System ready timeout - position data not available")
        return False
    
    def execute_mission(self):
        """Execute the survey mission using Aerostack2 actions"""
        try:
            self.get_logger().info("🚁 Starting Aerostack2 Survey Mission")
            
            if not self.wait_for_system_ready():
                return False
            
            # Phase 0: Wait for TF to be ready (ROS 2-compatible version of forum suggestion)
            self.get_logger().info("Phase 0: Waiting for TF readiness")
            if not self.wait_for_tf_ready(timeout_sec=15.0):
                self.get_logger().error("TF not ready - aborting mission")
                return False
            
            # Phase 1: Set mode and arm
            self.get_logger().info("Phase 1: Setting GUIDED mode and arming")
            if not self.set_guided_mode():
                return False
            
            # Check GPS status before arming (helpful for diagnosis)
            self.wait_for_gps(timeout_sec=10.0)  # Short timeout for SITL
            
            if not self.arm_drone_aerostack2():
                return False
            
            # Wait for platform to recognize armed state
            if not self.wait_for_armed_state(timeout_sec=5.0):
                self.get_logger().error("Platform did not report armed state - aborting mission")
                return False
            
            # Phase 2: Takeoff
            self.get_logger().info("Phase 2: Taking off")
            takeoff_altitude = self.config['altitude']
            if not self.takeoff_aerostack2(takeoff_altitude):
                return False
            
            # Phase 3: Execute survey pattern
            self.get_logger().info("Phase 3: Executing survey pattern")
            speed = self.config['flight_speed']
            
            for i, waypoint in enumerate(self.waypoints):
                self.get_logger().info(f"Waypoint {i+1}/{len(self.waypoints)}: {waypoint}")
                if not self.goto_waypoint(waypoint, speed):
                    self.get_logger().error(f"Failed to reach waypoint {i+1}")
                    return False
                
                # Small delay between waypoints
                time.sleep(1)
            
            # Phase 4: Land
            self.get_logger().info("Phase 4: Landing")
            if not self.land_aerostack2():
                return False
            
            self.get_logger().info("🎉 Survey mission completed successfully!")
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
                       default='/home/dhandavishal/aerostack2_ws/src/as2_ardu_msn/config/basic_survey_config.yaml',
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
