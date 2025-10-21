#!/usr/bin/env python3
"""
Simple test node to monitor multiple drone states
Subscribes to state topics from all drones and displays status
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState


class MultiDroneMonitor(Node):
    def __init__(self):
        super().__init__('multi_drone_monitor')
        
        # Get number of drones from parameter
        self.declare_parameter('num_drones', 3)
        self.num_drones = self.get_parameter('num_drones').value
        
        # Store state for each drone
        self.drone_states = {}
        self.drone_positions = {}
        self.drone_batteries = {}
        
        # QoS profile for MAVROS topics (best effort to match MAVROS)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers for each drone
        for i in range(self.num_drones):
            drone_ns = f'drone_{i}'
            
            # State subscriber
            self.create_subscription(
                State,
                f'/{drone_ns}/mavros/state',
                lambda msg, id=i: self.state_callback(msg, id),
                qos_profile
            )
            
            # Position subscriber
            self.create_subscription(
                PoseStamped,
                f'/{drone_ns}/mavros/local_position/pose',
                lambda msg, id=i: self.position_callback(msg, id),
                qos_profile
            )
            
            # Battery subscriber
            self.create_subscription(
                BatteryState,
                f'/{drone_ns}/mavros/battery',
                lambda msg, id=i: self.battery_callback(msg, id),
                qos_profile
            )
            
            # Initialize state
            self.drone_states[i] = None
            self.drone_positions[i] = None
            self.drone_batteries[i] = None
        
        # Create timer for periodic status display
        self.timer = self.create_timer(2.0, self.display_status)
        
        self.get_logger().info(f'Multi-Drone Monitor initialized for {self.num_drones} drones')
    
    def state_callback(self, msg, drone_id):
        self.drone_states[drone_id] = msg
    
    def position_callback(self, msg, drone_id):
        self.drone_positions[drone_id] = msg
    
    def battery_callback(self, msg, drone_id):
        self.drone_batteries[drone_id] = msg
    
    def display_status(self):
        """Display status of all drones"""
        print("\n" + "="*80)
        print("Multi-Drone Status Monitor")
        print("="*80)
        
        for i in range(self.num_drones):
            print(f"\n🚁 Drone {i}:")
            
            # State
            state = self.drone_states.get(i)
            if state:
                connected = "✓" if state.connected else "✗"
                armed = "✓" if state.armed else "✗"
                mode = state.mode
                print(f"   Connected: {connected} | Armed: {armed} | Mode: {mode}")
            else:
                print("   State: No data")
            
            # Position
            pos = self.drone_positions.get(i)
            if pos:
                x = pos.pose.position.x
                y = pos.pose.position.y
                z = pos.pose.position.z
                print(f"   Position: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
            else:
                print("   Position: No data")
            
            # Battery
            battery = self.drone_batteries.get(i)
            if battery:
                voltage = battery.voltage
                percentage = battery.percentage * 100 if battery.percentage >= 0 else 0
                print(f"   Battery: {voltage:.2f}V ({percentage:.1f}%)")
            else:
                print("   Battery: No data")
        
        print("\n" + "="*80)


def main(args=None):
    rclpy.init(args=args)
    
    monitor = MultiDroneMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
