#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool              # arming service

class ArmNode(Node):
    def __init__(self):
        super().__init__('arm_node')
        self.cli = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.timer = self.create_timer(1.0, self.try_arm)

    def try_arm(self):
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('Waiting for /mavros/cmd/arming...')
            return

        req = CommandBool.Request()
        req.value = True
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info('✓ Drone armed')
        else:
            self.get_logger().error(f'✗ Arming failed: {future.result()}')
        rclpy.shutdown()                 # stop once arming attempt finishes


def main():
    rclpy.init()
    node = ArmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
