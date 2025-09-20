#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from math import radians, cos, sin, sqrt, atan2, degrees
from typing import List, Tuple
import time

from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import (
    State, ExtendedState, HomePosition, 
    PositionTarget, GlobalPositionTarget
)
from mavros_msgs.srv import (
    CommandBool, CommandTOL, SetMode,
    CommandLong, WaypointPush, WaypointClear
)
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Header, Float64

class SurveyMissionNode(Node):
    def __init__(self):
        super().__init__('survey_mission_node')
        
        # Load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('survey.area.corner1', [47.397742, 8.545594]),
                ('survey.area.corner2', [47.398242, 8.545594]),
                ('survey.area.corner3', [47.398242, 8.546594]),
                ('survey.area.corner4', [47.397742, 8.546594]),
                ('survey.altitude', 50.0),
                ('survey.speed', 5.0),
                ('survey.line_spacing', 20.0),
                ('survey.overshoot', 10.0),
                ('survey.camera.trigger_distance', 5.0),
                ('survey.camera.gimbal_pitch', -90.0),
                ('survey.safety.rtl_altitude', 60.0),
                ('survey.safety.geofence_radius', 200.0),
                ('survey.safety.min_battery', 20.0),
            ]
        )
        
        # Get parameters
        self.survey_corners = [
            self.get_parameter('survey.area.corner1').value,
            self.get_parameter('survey.area.corner2').value,
            self.get_parameter('survey.area.corner3').value,
            self.get_parameter('survey.area.corner4').value,
        ]
        self.survey_altitude = self.get_parameter('survey.altitude').value
        self.survey_speed = self.get_parameter('survey.speed').value
        self.line_spacing = self.get_parameter('survey.line_spacing').value
        self.overshoot = self.get_parameter('survey.overshoot').value
        self.trigger_distance = self.get_parameter('survey.camera.trigger_distance').value
        self.gimbal_pitch = self.get_parameter('survey.camera.gimbal_pitch').value
        self.rtl_altitude = self.get_parameter('survey.safety.rtl_altitude').value
        self.geofence_radius = self.get_parameter('survey.safety.geofence_radius').value
        self.min_battery = self.get_parameter('survey.safety.min_battery').value
        
        # State variables
        self.current_state = None
        self.current_gps = None
        self.current_local_pos = None
        self.home_position = None
        self.battery_state = None
        self.is_armed = False
        self.in_guided_mode = False
        self.mission_started = False
        self.current_waypoint_index = 0
        
        # QoS profile for subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/drone0/mavros/state', self.state_callback, qos
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/drone0/mavros/global_position/global', 
            self.gps_callback, qos
        )
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/drone0/mavros/local_position/pose',
            self.local_position_callback, qos
        )
        self.home_pos_sub = self.create_subscription(
            HomePosition, '/drone0/mavros/home_position/home',
            self.home_position_callback, qos
        )
        self.battery_sub = self.create_subscription(
            BatteryState, '/drone0/mavros/battery',
            self.battery_callback, qos
        )
        
        # Publishers for ArduPilot GUIDED mode
        self.guided_global_pub = self.create_publisher(
            GlobalPositionTarget, 
            '/drone0/mavros/guided_target/global', 
            10
        )
        self.guided_local_pub = self.create_publisher(
            PositionTarget,
            '/drone0/mavros/setpoint_raw/local',
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/drone0/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/drone0/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/drone0/mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, '/drone0/mavros/set_mode')
        self.command_client = self.create_client(CommandLong, '/drone0/mavros/cmd/command')

        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.arming_client.wait_for_service(timeout_sec=30.0)
        self.takeoff_client.wait_for_service(timeout_sec=30.0)
        self.set_mode_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('MAVROS services available!')

        # Generate survey waypoints
        self.survey_waypoints = self.generate_survey_pattern()
        self.get_logger().info(f'Generated {len(self.survey_waypoints)} survey waypoints')

        # Start mission timer
        self.mission_timer = self.create_timer(1.0, self.mission_loop)
        
    def state_callback(self, msg):
        self.current_state = msg
        self.is_armed = msg.armed
        self.in_guided_mode = (msg.mode == 'GUIDED')
        
    def gps_callback(self, msg):
        self.current_gps = msg
        
    def local_position_callback(self, msg):
        self.current_local_pos = msg
        
    def home_position_callback(self, msg):
        self.home_position = msg
        
    def battery_callback(self, msg):
        self.battery_state = msg
        
    def generate_survey_pattern(self) -> List[Tuple[float, float, float]]:
        """Generate lawnmower survey pattern from corner points"""
        waypoints = []
        
        # Convert corners to local coordinates (simplified - use proper projection in production)
        # This is a basic implementation - you should use proper UTM or local tangent plane
        corners_local = []
        ref_lat = self.survey_corners[0][0]
        ref_lon = self.survey_corners[0][1]
        
        for lat, lon in self.survey_corners:
            # Simple equirectangular projection (good for small areas)
            x = (lon - ref_lon) * 111320.0 * cos(radians(ref_lat))
            y = (lat - ref_lat) * 110540.0
            corners_local.append((x, y))
        
        # Calculate survey lines (simple lawnmower pattern)
        # Find bounding box
        min_x = min(c[0] for c in corners_local)
        max_x = max(c[0] for c in corners_local)
        min_y = min(c[1] for c in corners_local)
        max_y = max(c[1] for c in corners_local)
        
        # Generate parallel lines
        current_x = min_x
        line_num = 0
        
        while current_x <= max_x:
            if line_num % 2 == 0:
                # Even lines: go from bottom to top
                start_y = min_y - self.overshoot
                end_y = max_y + self.overshoot
                y_step = self.trigger_distance
            else:
                # Odd lines: go from top to bottom
                start_y = max_y + self.overshoot
                end_y = min_y - self.overshoot
                y_step = -self.trigger_distance
            
            # Add waypoints along the line
            current_y = start_y
            while (y_step > 0 and current_y <= end_y) or (y_step < 0 and current_y >= end_y):
                # Convert back to GPS coordinates
                lat = ref_lat + (current_y / 110540.0)
                lon = ref_lon + (current_x / (111320.0 * cos(radians(ref_lat))))
                waypoints.append((lat, lon, self.survey_altitude))
                current_y += y_step
            
            current_x += self.line_spacing
            line_num += 1
        
        return waypoints
    
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode"""
        req = SetMode.Request()
        req.custom_mode = mode_name
        
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().mode_sent
        return False
    
    def arm_vehicle(self) -> bool:
        """Arm the vehicle"""
        req = CommandBool.Request()
        req.value = True
        
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude"""
        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = self.current_gps.latitude if self.current_gps else 0.0
        req.longitude = self.current_gps.longitude if self.current_gps else 0.0
        
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            return future.result().success
        return False
    
    def send_global_position_target(self, lat: float, lon: float, alt: float):
        """Send position target for ArduPilot GUIDED mode"""
        msg = GlobalPositionTarget()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX |
            GlobalPositionTarget.IGNORE_VY |
            GlobalPositionTarget.IGNORE_VZ |
            GlobalPositionTarget.IGNORE_AFX |
            GlobalPositionTarget.IGNORE_AFY |
            GlobalPositionTarget.IGNORE_AFZ |
            GlobalPositionTarget.IGNORE_YAW |
            GlobalPositionTarget.IGNORE_YAW_RATE
        )
        
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        
        self.guided_global_pub.publish(msg)
    
    def check_waypoint_reached(self, target_lat: float, target_lon: float, threshold: float = 3.0) -> bool:
        """Check if current position is within threshold of target"""
        if not self.current_gps:
            return False
        
        # Calculate distance using haversine formula
        R = 6371000  # Earth radius in meters
        lat1 = radians(self.current_gps.latitude)
        lat2 = radians(target_lat)
        dlat = radians(target_lat - self.current_gps.latitude)
        dlon = radians(target_lon - self.current_gps.longitude)
        
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        distance = R * c
        
        return distance < threshold
    
    def check_battery(self) -> bool:
        """Check if battery level is safe"""
        if self.battery_state:
            return self.battery_state.percentage * 100 > self.min_battery
        return True
    
    def mission_loop(self):
        """Main mission control loop"""
        if not self.current_state:
            return
        
        # Safety checks
        if not self.check_battery():
            self.get_logger().warn('Low battery! Initiating RTL')
            self.set_mode('RTL')
            return
        
        # Mission state machine
        if not self.mission_started:
            # Pre-flight checks
            if not self.in_guided_mode:
                self.get_logger().info('Setting GUIDED mode...')
                if self.set_mode('GUIDED'):
                    self.get_logger().info('GUIDED mode set!')
                    time.sleep(2)
            
            elif not self.is_armed:
                self.get_logger().info('Arming vehicle...')
                if self.arm_vehicle():
                    self.get_logger().info('Vehicle armed!')
                    time.sleep(2)
            
            elif self.current_gps and self.current_gps.altitude < self.survey_altitude - 5:
                self.get_logger().info(f'Taking off to {self.survey_altitude}m...')
                if self.takeoff(self.survey_altitude):
                    self.get_logger().info('Takeoff command sent!')
                    time.sleep(5)
            
            else:
                self.get_logger().info('Starting survey mission!')
                self.mission_started = True
        
        else:
            # Execute survey mission
            if self.current_waypoint_index < len(self.survey_waypoints):
                wp = self.survey_waypoints[self.current_waypoint_index]
                
                # Send position target
                self.send_global_position_target(wp[0], wp[1], wp[2])
                
                # Check if waypoint reached
                if self.check_waypoint_reached(wp[0], wp[1]):
                    self.get_logger().info(
                        f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.survey_waypoints)}'
                    )
                    
                    # Trigger camera (send MAVLink command)
                    self.trigger_camera()
                    
                    self.current_waypoint_index += 1
            
            else:
                # Mission complete
                self.get_logger().info('Survey mission complete! Landing...')
                self.set_mode('LAND')
                self.mission_started = False
    
    def trigger_camera(self):
        """Send camera trigger command"""
        req = CommandLong.Request()
        req.command = 203  # MAV_CMD_DO_DIGICAM_CONTROL
        req.param1 = 1.0   # Trigger camera
        
        future = self.command_client.call_async(req)
        # Fire and forget - don't wait for response

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SurveyMissionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()