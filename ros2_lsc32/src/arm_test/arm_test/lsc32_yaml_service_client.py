#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from arm_servie.srv import MultiServo  # Generated from srv/MultiServo.srv
import yaml
import sys
import os
import time

class LSC32YAMLServiceClient(Node):
    """
    ROS2 client node that reads servo commands from a YAML file and sends them
    sequentially to the LSC-32 via the MultiServo service.
    YAML format:
      - ids: [1, 2]
        positions: [500, 600]
        time_ms: 1500
      - ids: [1, 2]
        positions: [300, 700]
        time_ms: 1000
    """

    def __init__(self, yaml_path):
        super().__init__('lsc32_yaml_service_client')
        self.cli = self.create_client(MultiServo, 'move_servos')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for move_servos service...')

        # Load YAML commands
        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"YAML file not found: {yaml_path}")
            sys.exit(1)

        try:
            with open(yaml_path, 'r') as f:
                self.commands = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {e}")
            sys.exit(1)

        if not isinstance(self.commands, list):
            self.get_logger().error("YAML file must contain a list of commands")
            sys.exit(1)

        self.get_logger().info(f"Loaded {len(self.commands)} commands from {yaml_path}")

    def send_request(self, ids, positions, time_ms):
        """Send a request to move multiple servos."""
        req = MultiServo.Request()
        req.ids = ids
        req.positions = positions
        req.time_ms = time_ms
        return self.cli.call_async(req)

    def execute_sequence(self):
        """Send all commands from YAML sequentially."""
        for idx, cmd in enumerate(self.commands, start=1):
            try:
                ids = cmd['ids']
                positions = cmd['positions']
                time_ms = cmd['time_ms']
            except KeyError as e:
                self.get_logger().error(f"Command {idx} missing key: {e}")
                continue

            self.get_logger().info(f"Sending command {idx}: IDs={ids}, POS={positions}, TIME={time_ms}ms")
            future = self.send_request(ids, positions, time_ms)
            rclpy.spin_until_future_complete(self, future)

            if future.result() is not None:
                res = future.result()
                if res.success:
                    self.get_logger().info(f"? Success: {res.message}")
                else:
                    self.get_logger().error(f"? Failed: {res.message}")
            else:
                self.get_logger().error(f"Service call for command {idx} failed or timed out.")

            # Wait for the move to complete before sending next command
            time.sleep(time_ms / 1000.0)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: ros2 run your_package lsc32_yaml_service_client <path_to_yaml>")
        sys.exit(1)

    yaml_path = sys.argv[1]
    node = LSC32YAMLServiceClient(yaml_path)

    try:
        node.execute_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
