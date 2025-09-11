#!/usr/bin/env python3

"""
Clear mission from ArduPilot SITL to resolve 'Mission is stale' issue
"""

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointClear, WaypointPush
from mavros_msgs.msg import Waypoint
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import time

class MissionCleaner(Node):
    def __init__(self):
        super().__init__('mission_cleaner')
        
        # Wait for MAVROS services
        self.clear_client = self.create_client(WaypointClear, '/mavros/mission/clear')
        self.push_client = self.create_client(WaypointPush, '/mavros/mission/push')
        
        self.get_logger().info("Waiting for MAVROS mission services...")
        
        # Wait for services to be available
        while not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission/clear service...')
        
        while not self.push_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for mission/push service...')
            
        self.get_logger().info("✓ MAVROS mission services available")
    
    def clear_mission(self):
        """Clear all waypoints from autopilot"""
        self.get_logger().info("Clearing mission from autopilot...")
        
        request = WaypointClear.Request()
        future = self.clear_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("✓ Mission cleared successfully")
                return True
            else:
                self.get_logger().error("Failed to clear mission")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False
    
    def upload_home_waypoint(self):
        """Upload a single HOME waypoint to establish a clean mission"""
        self.get_logger().info("Uploading clean HOME waypoint...")
        
        # Create HOME waypoint (CMD_NAV_WAYPOINT at current position)
        waypoint = Waypoint()
        waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
        waypoint.command = 16  # MAV_CMD_NAV_WAYPOINT
        waypoint.is_current = True  # This is the current/active waypoint
        waypoint.autocontinue = True
        waypoint.param1 = 0.0  # Hold time
        waypoint.param2 = 0.0  # Acceptance radius
        waypoint.param3 = 0.0  # Pass radius
        waypoint.param4 = 0.0  # Yaw angle
        waypoint.x_lat = -35.363261  # SITL home latitude
        waypoint.y_long = 149.165230  # SITL home longitude
        waypoint.z_alt = 10.0  # 10m above home
        
        request = WaypointPush.Request()
        request.start_index = 0
        request.waypoints = [waypoint]
        
        future = self.push_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("✓ Clean HOME waypoint uploaded")
                return True
            else:
                self.get_logger().error(f"Failed to upload waypoint: wp_transfered={future.result().wp_transfered}")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False
    
    def run_cleanup(self):
        """Run the complete mission cleanup process"""
        self.get_logger().info("Starting mission cleanup to resolve 'Mission is stale' issue...")
        
        # Step 1: Clear existing mission
        if not self.clear_mission():
            return False
        
        # Small delay to let ArduPilot process the clear
        time.sleep(1.0)
        
        # Step 2: Upload a clean HOME waypoint
        if not self.upload_home_waypoint():
            return False
        
        self.get_logger().info("✓ Mission cleanup complete!")
        self.get_logger().info("The 'Mission is stale' issue should now be resolved.")
        self.get_logger().info("You can now run your survey mission.")
        
        return True

def main():
    rclpy.init()
    
    cleaner = MissionCleaner()
    
    try:
        success = cleaner.run_cleanup()
        if success:
            print("\n✅ Mission cleanup successful!")
            print("   - Old mission cleared from ArduPilot")
            print("   - Clean HOME waypoint uploaded")
            print("   - 'Mission is stale' issue resolved")
        else:
            print("\n❌ Mission cleanup failed!")
            return 1
            
    except KeyboardInterrupt:
        cleaner.get_logger().info("Mission cleanup interrupted")
        return 1
    finally:
        cleaner.destroy_node()
        rclpy.shutdown()
    
    return 0

if __name__ == '__main__':
    exit(main())
