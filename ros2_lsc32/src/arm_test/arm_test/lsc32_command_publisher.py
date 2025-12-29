#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import yaml
import os
import sys

class LSC32CommandPublisher(Node):
    """
    Publishes multi-servo commands to /servo_cmd for the LSC-32 controller.
    Message format: [id1, pos1, id2, pos2, ..., time_ms]
    """

    def __init__(self, yaml_path=None, publish_rate=1.0):
        super().__init__('lsc32_command_publisher')

        self.publisher_ = self.create_publisher(Int32MultiArray, '/servo_cmd', 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        self.commands = []
        self.index = 0

        if yaml_path and os.path.isfile(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    data = yaml.safe_load(f)
                if isinstance(data, list):
                    self.commands = data
                    self.get_logger().info(f"Loaded {len(self.commands)} commands from {yaml_path}")
                else:
                    self.get_logger().error("YAML file must contain a list of commands")
            except Exception as e:
                self.get_logger().error(f"Failed to load YAML file: {e}")
        else:
            # Default test sequence if no YAML provided
            self.get_logger().warn("No YAML file provided  using default test sequence")
            self.commands = [
                [1, 500, 2, 500, 1000],  # Move servo 1 & 2 to 500 in 1s
                [1, 300, 2, 700, 1000],  # Move servo 1 & 2 to new positions in 1s
                [1, 500, 2, 500, 1000]   # Return to center
            ]

    def timer_callback(self):
        if not self.commands:
            return

        msg = Int32MultiArray()
        msg.data = self.commands[self.index]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {msg.data}")

        self.index = (self.index + 1) % len(self.commands)


def main(args=None):
    rclpy.init(args=args)

    yaml_path = None
    if len(sys.argv) > 1:
        yaml_path = sys.argv[1]

    node = LSC32CommandPublisher(yaml_path=yaml_path, publish_rate=0.5)  # 0.5 Hz = every 2s
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
