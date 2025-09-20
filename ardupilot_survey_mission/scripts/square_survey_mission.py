#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import math

from mavros_msgs.msg import State, GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped

class SquareSurveyMission(Node):
    def __init__(self):
        super().__init__('square_survey_mission')
        
        # Mission parameters
        self.square_size = 20.0  # meters per side
        self.survey_altitude = 10.0  # meters
        self.waypoint_threshold = 3.0  # meters - how close to consider waypoint reached
        
        # State variables
        self.current_state = None
        self.current_gps = None
        self.current_local_pos = None
        self.home_position = None
        self.mission_step = 0
        self.current_waypoint_index = 0
        self.waypoints = []
        self.takeoff_time = None
        self.hover_start_time = None
        
        # QoS profile for MAVROS compatibility
        mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, mavros_qos
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/mavros/global', 
            self.gps_callback, mavros_qos
        )
        self.local_pos_sub = self.create_subscription(
            PoseStamped, '/mavros/mavros/local',
            self.local_position_callback, mavros_qos
        )
        
        # Publishers for navigation
        self.global_target_pub = self.create_publisher(
            GlobalPositionTarget,
            '/mavros/mavros/set_global',
            10
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/mavros/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/mavros/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/mavros/land')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        self.wait_for_services()
        self.get_logger().info('All MAVROS services available!')
        
        # Mission timer
        self.mission_timer = self.create_timer(2.0, self.mission_loop)
        
    def wait_for_services(self):
        """Wait for all required services to be available"""
        services = [
            (self.arming_client, '/mavros/mavros/arming'),
            (self.takeoff_client, '/mavros/mavros/takeoff'),
            (self.land_client, '/mavros/mavros/land'),
            (self.set_mode_client, '/mavros/set_mode')
        ]
        
        for client, service_name in services:
            if not client.wait_for_service(timeout_sec=30.0):
                self.get_logger().error(f'Service {service_name} not available!')
                raise RuntimeError(f'Service {service_name} not available')
            else:
                self.get_logger().info(f'Service {service_name} is ready')
        
    def state_callback(self, msg):
        """Handle state updates from MAVROS"""
        self.current_state = msg
        
    def gps_callback(self, msg):
        """Handle GPS updates from MAVROS"""
        self.current_gps = msg
        
    def local_position_callback(self, msg):
        """Handle local position updates from MAVROS"""
        self.current_local_pos = msg
        
    def generate_square_waypoints(self):
        """Generate square pattern waypoints relative to current position"""
        if not self.current_gps:
            self.get_logger().error('No GPS position available for waypoint generation')
            return
            
        # Use current GPS position as center of square
        center_lat = self.current_gps.latitude
        center_lon = self.current_gps.longitude
        
        # Convert meters to approximate lat/lon offsets
        # Rough approximation: 1 degree lat ≈ 111,000m, 1 degree lon ≈ 111,000m * cos(lat)
        lat_per_meter = 1.0 / 111000.0
        lon_per_meter = 1.0 / (111000.0 * math.cos(math.radians(center_lat)))
        
        # Half the square size for offsets from center
        half_size = self.square_size / 2.0
        
        # Define square corners (clockwise from northeast)
        self.waypoints = [
            # Northeast corner
            (center_lat + half_size * lat_per_meter, 
             center_lon + half_size * lon_per_meter, 
             self.survey_altitude),
            # Southeast corner  
            (center_lat - half_size * lat_per_meter,
             center_lon + half_size * lon_per_meter,
             self.survey_altitude),
            # Southwest corner
            (center_lat - half_size * lat_per_meter,
             center_lon - half_size * lon_per_meter,
             self.survey_altitude),
            # Northwest corner
            (center_lat + half_size * lat_per_meter,
             center_lon - half_size * lon_per_meter,
             self.survey_altitude),
            # Return to northeast (close the square)
            (center_lat + half_size * lat_per_meter,
             center_lon + half_size * lon_per_meter,
             self.survey_altitude),
        ]
        
        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints for {self.square_size}m square')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i}: lat={wp[0]:.7f}, lon={wp[1]:.7f}, alt={wp[2]:.1f}m')
            
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode"""
        req = SetMode.Request()
        req.custom_mode = mode_name
        
        self.get_logger().info(f'Setting mode to {mode_name}...')
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            success = future.result().mode_sent
            if success:
                self.get_logger().info(f'Mode {mode_name} set successfully')
            else:
                self.get_logger().error(f'Failed to set mode {mode_name}')
            return success
        else:
            self.get_logger().warn(f'Service call timeout for mode {mode_name}, but checking state...')
            return False
    
    def arm_vehicle(self) -> bool:
        """Arm the vehicle"""
        req = CommandBool.Request()
        req.value = True
        
        self.get_logger().info('Arming vehicle...')
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            success = future.result().success
            if success:
                self.get_logger().info('Vehicle armed successfully')
            else:
                self.get_logger().error('Failed to arm vehicle')
            return success
        else:
            self.get_logger().warn('Service call timeout for arming, but checking state...')
            return False
    
    def takeoff(self, altitude: float) -> bool:
        """Takeoff to specified altitude"""
        req = CommandTOL.Request()
        req.altitude = altitude
        if self.current_gps:
            req.latitude = self.current_gps.latitude
            req.longitude = self.current_gps.longitude
        else:
            req.latitude = 0.0
            req.longitude = 0.0
        
        self.get_logger().info(f'Taking off to {altitude}m...')
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        
        if future.result() is not None:
            success = future.result().success
            if success:
                self.get_logger().info(f'Takeoff to {altitude}m initiated')
            else:
                self.get_logger().error('Takeoff failed')
            return success
        else:
            self.get_logger().warn('Service call timeout for takeoff, but command may have been sent')
            return True
    
    def land(self) -> bool:
        """Land the vehicle"""
        req = CommandTOL.Request()
        req.altitude = 0.0
        if self.current_gps:
            req.latitude = self.current_gps.latitude
            req.longitude = self.current_gps.longitude
        else:
            req.latitude = 0.0
            req.longitude = 0.0
        
        self.get_logger().info('Landing...')
        future = self.land_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            success = future.result().success
            if success:
                self.get_logger().info('Landing initiated')
            else:
                self.get_logger().error('Landing failed')
            return success
        else:
            self.get_logger().error('Service call timeout for landing')
            return False
            
    def send_waypoint(self, lat: float, lon: float, alt: float):
        """Send a waypoint as a global position target"""
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Set coordinate frame and type mask
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
        
        self.global_target_pub.publish(msg)
        
    def calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS coordinates using haversine formula"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
        
    def is_waypoint_reached(self, target_lat: float, target_lon: float) -> bool:
        """Check if current position is within threshold of target waypoint"""
        if not self.current_gps:
            return False
            
        distance = self.calculate_distance(
            self.current_gps.latitude, self.current_gps.longitude,
            target_lat, target_lon
        )
        
        return distance < self.waypoint_threshold
    
    def mission_loop(self):
        """Main mission control loop"""
        if not self.current_state:
            self.get_logger().info('Waiting for MAVROS state...')
            return
        
        self.get_logger().info(f'Mission step: {self.mission_step}, '
                              f'Mode: {self.current_state.mode}, '
                              f'Armed: {self.current_state.armed}, '
                              f'Connected: {self.current_state.connected}')
        
        if self.mission_step == 0:
            # Step 0: Check connection
            if self.current_state.connected:
                self.get_logger().info('MAVROS connected to FCU!')
                self.mission_step = 1
            else:
                self.get_logger().info('Waiting for MAVROS connection...')
                
        elif self.mission_step == 1:
            # Step 1: Set GUIDED mode
            if self.current_state.mode != 'GUIDED':
                self.get_logger().info(f'Current mode: {self.current_state.mode}, requesting GUIDED mode...')
                self.set_mode('GUIDED')
                time.sleep(1.0)
            else:
                self.get_logger().info('Successfully in GUIDED mode!')
                self.mission_step = 2
                
        elif self.mission_step == 2:
            # Step 2: Arm vehicle and immediately send takeoff
            if not self.current_state.armed:
                self.get_logger().info('Vehicle not armed, sending arm command...')
                self.arm_vehicle()
                self.get_logger().info('Arm command sent, immediately sending takeoff to prevent auto-disarm...')
                time.sleep(1.0)
                self.takeoff(self.survey_altitude)
                self.mission_step = 3
                self.takeoff_time = time.time()
            else:
                self.get_logger().info('Vehicle successfully armed!')
                self.mission_step = 3
                
        elif self.mission_step == 3:
            # Step 3: Monitor takeoff progress and generate waypoints
            if hasattr(self, 'takeoff_time'):
                elapsed = time.time() - self.takeoff_time
                if elapsed < 15.0:
                    self.get_logger().info(f'Takeoff in progress... {elapsed:.1f}s elapsed')
                    # Generate waypoints once we have GPS
                    if not self.waypoints and self.current_gps:
                        self.generate_square_waypoints()
                else:
                    self.get_logger().info('Takeoff complete, starting square survey mission')
                    if not self.waypoints:
                        self.generate_square_waypoints()
                    self.mission_step = 4
                    self.current_waypoint_index = 0
            else:
                # If we somehow got here without takeoff_time, send takeoff command
                self.get_logger().info('Sending takeoff command...')
                self.takeoff(self.survey_altitude)
                self.takeoff_time = time.time()
                
        elif self.mission_step == 4:
            # Step 4: Execute square survey pattern
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]
                target_lat, target_lon, target_alt = waypoint
                
                # Send waypoint command
                self.send_waypoint(target_lat, target_lon, target_alt)
                
                # Check if waypoint reached
                if self.is_waypoint_reached(target_lat, target_lon):
                    self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}')
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index >= len(self.waypoints):
                        self.get_logger().info('Square survey pattern complete!')
                        self.mission_step = 5
                else:
                    if self.current_gps:
                        distance = self.calculate_distance(
                            self.current_gps.latitude, self.current_gps.longitude,
                            target_lat, target_lon
                        )
                        self.get_logger().info(f'Flying to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}, '
                                             f'distance: {distance:.1f}m')
            else:
                self.get_logger().info('All waypoints completed!')
                self.mission_step = 5
                
        elif self.mission_step == 5:
            # Step 5: Land
            self.get_logger().info('Landing...')
            self.land()
            self.mission_step = 6
            
        elif self.mission_step == 6:
            # Step 6: Mission complete
            self.get_logger().info('=== SQUARE SURVEY MISSION COMPLETE ===')
            self.mission_timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SquareSurveyMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nMission interrupted by user')
    except Exception as e:
        print(f'\nMission failed with error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
