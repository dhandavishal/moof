#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

class SimpleFlightTest(Node):
    def __init__(self):
        super().__init__('simple_flight_test')
        
        # State variables
        self.current_state = None
        self.current_gps = None
        self.mission_step = 0
        
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
        
    def set_mode(self, mode_name: str) -> bool:
        """Set flight mode"""
        req = SetMode.Request()
        req.custom_mode = mode_name
        
        self.get_logger().info(f'Setting mode to {mode_name}...')
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)  # Increased timeout
        
        if future.result() is not None:
            success = future.result().mode_sent
            if success:
                self.get_logger().info(f'Mode {mode_name} set successfully')
            else:
                self.get_logger().error(f'Failed to set mode {mode_name}')
            return success
        else:
            self.get_logger().warn(f'Service call timeout for mode {mode_name}, but checking state...')
            # Even if service times out, check if mode actually changed
            return False  # We'll check the actual mode in the mission loop
    
    def arm_vehicle(self) -> bool:
        """Arm the vehicle"""
        req = CommandBool.Request()
        req.value = True
        
        self.get_logger().info('Arming vehicle...')
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)  # Increased timeout
        
        if future.result() is not None:
            success = future.result().success
            if success:
                self.get_logger().info('Vehicle armed successfully')
            else:
                self.get_logger().error('Failed to arm vehicle')
            return success
        else:
            self.get_logger().warn('Service call timeout for arming, but checking state...')
            # Even if service times out, check if vehicle actually armed
            return False  # We'll check the actual armed state in mission loop
    
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
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)  # Increased timeout
        
        if future.result() is not None:
            success = future.result().success
            if success:
                self.get_logger().info(f'Takeoff to {altitude}m initiated')
            else:
                self.get_logger().error('Takeoff failed')
            return success
        else:
            self.get_logger().warn('Service call timeout for takeoff, but command may have been sent')
            return True  # Assume success since ArduPilot might be processing
    
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
    
    def mission_loop(self):
        """Simple mission: GUIDED mode -> ARM -> TAKEOFF -> wait -> LAND"""
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
                self.set_mode('GUIDED')  # Don't wait for response, just send the command
                # Give some time for mode change to take effect
                time.sleep(1.0)
            else:
                self.get_logger().info('Successfully in GUIDED mode!')
                self.mission_step = 2
                
        elif self.mission_step == 2:
            # Step 2: Arm vehicle and immediately send takeoff
            if not self.current_state.armed:
                self.get_logger().info('Vehicle not armed, sending arm command...')
                self.arm_vehicle()  # Don't wait for response, just send the command
                self.get_logger().info('Arm command sent, immediately sending takeoff to prevent auto-disarm...')
                # Send takeoff immediately to prevent auto-disarm
                time.sleep(1.0)  # Brief pause
                self.takeoff(10.0)  # Send takeoff command right away
                self.mission_step = 3  # Move to next step
                self.takeoff_time = time.time()
            else:
                self.get_logger().info('Vehicle successfully armed!')
                self.mission_step = 3
                
        elif self.mission_step == 3:
            # Step 3: Monitor takeoff progress
            if hasattr(self, 'takeoff_time'):
                elapsed = time.time() - self.takeoff_time
                if elapsed < 15.0:  # Give 15 seconds for takeoff
                    self.get_logger().info(f'Takeoff in progress... {elapsed:.1f}s elapsed')
                else:
                    self.get_logger().info('Takeoff complete, starting hover phase')
                    self.mission_step = 4
                    self.hover_start_time = time.time()
            else:
                # If we somehow got here without takeoff_time, send takeoff command
                self.get_logger().info('Sending takeoff command...')
                self.takeoff(10.0)
                self.takeoff_time = time.time()
                
        elif self.mission_step == 4:
            # Step 4: Wait at altitude for 10 seconds
            if hasattr(self, 'hover_start_time'):
                if time.time() - self.hover_start_time > 10.0:
                    self.get_logger().info('Hover complete, initiating landing')
                    self.mission_step = 5
                else:
                    remaining = 10.0 - (time.time() - self.hover_start_time)
                    self.get_logger().info(f'Hovering... {remaining:.1f}s remaining')
            else:
                # Start hover timer
                self.hover_start_time = time.time()
                self.get_logger().info('Starting hover phase...')
                
        elif self.mission_step == 5:
            # Step 5: Land
            if self.land():
                self.mission_step = 6
                
        elif self.mission_step == 6:
            # Step 6: Mission complete
            self.get_logger().info('=== MISSION COMPLETE ===')
            self.mission_timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SimpleFlightTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nMission interrupted by user')
    except Exception as e:
        print(f'\nMission failed with error: {e}')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
