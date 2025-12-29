#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_servie.srv import MultiServo  # Generated from srv/MultiServo.srv

class LSC32ServiceClient(Node):
    """
    ROS2 client node to send multi-servo commands to the LSC-32 via the MultiServo service.
    """

    def __init__(self):
        super().__init__('lsc32_service_client')
        self.cli = self.create_client(MultiServo, 'move_servos')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_servos service...')

        self.req = MultiServo.Request()

    def send_request(self, ids, positions, time_ms):
        """Send a request to move multiple servos."""
        self.req.ids = ids
        self.req.positions = positions
        self.req.time_ms = time_ms
        return self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    node = LSC32ServiceClient()

    # Example: Move servo 1 to 500, servo 2 to 600 in 1500ms
    ids = [1, 2]
    positions = [500, 600]
    time_ms = 1500

    future = node.send_request(ids, positions, time_ms)

    # Wait for the response
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        res = future.result()
        if res.success:
            node.get_logger().info(f"? Success: {res.message}")
        else:
            node.get_logger().error(f"? Failed: {res.message}")
    else:
        node.get_logger().error("Service call failed or timed out.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
