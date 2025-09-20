#!/usr/bin/env python3

import rclpy
from as2_python_api.drone_interface import DroneInterface
import time


def test_aerostack_arm():
    print("=" * 50)
    print("Aerostack2 Arming Test")
    print("=" * 50)

    rclpy.init()

    try:
        drone = DroneInterface('drone0', use_sim_time=False, verbose=True)

        print("1. Creating drone interface finished")

        max_wait = 20
        start_time = time.time()
        connected = False

        print("2. Waiting for connection...")
        while time.time() - start_time < max_wait:
            info = drone.info
            print(f" Connected: {info['connected']}, Armed: {info['armed']}, State: {info['state']}")
            if info['connected']:
                print(" ✓ Connected to drone!")
                connected = True
                break
            time.sleep(1)

        if not connected:
            print(" ✗ Failed to connect to drone")
            return False

        print("3. Waiting for system to stabilize...")
        time.sleep(3)

        print("4. Setting offboard mode...")
        if drone.offboard():
            print(" ✓ Offboard mode set")
        else:
            print(" ✗ Failed to set offboard mode")
            return False

        time.sleep(2)

        print("5. Arming drone...")
        if drone.arm():
            print(" ✓ Drone armed!")
        else:
            print(" ✗ Failed to arm drone")
            return False

        print("6. Drone is armed! Waiting 5 seconds...")
        time.sleep(5)

        print("7. Disarming drone...")
        if drone.disarm():
            print(" ✓ Drone disarmed")
        else:
            print(" ✗ Failed to disarm drone")

        print("8. Setting manual mode...")
        if drone.manual():
            print(" ✓ Manual mode set")
        else:
            print(" ✗ Failed to set manual mode")

        drone.shutdown()
        print("" + "="*50)
        print("Test completed successfully!")
        print("="*50)

        return True

    except Exception as e:
        print(f"Exception caught: {e}")
        return False

    finally:
        # Shutdown rclpy here ONLY if it has not yet been shut down
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    test_aerostack_arm()
