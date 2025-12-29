#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import sys

class LSC32TopicController(Node):
    """
    ROS2 node that listens to /servo_cmd topic and sends commands to LSC-32.
    Message format: Int32MultiArray with [servo_id, position, time_ms]
    Example: data=[1, 500, 1000] -> Move servo 1 to pos 500 in 1000ms
    """

    def __init__(self, port='/dev/ttyTHS1', baudrate=9600):
        super().__init__('lsc32_topic_controller')

        # Connect to LSC-32
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to LSC-32 on {port} at {baudrate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            sys.exit(1)

        # Subscribe to servo command topic
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_cmd',
            self.servo_cmd_callback,
            10
        )

    def servo_cmd_callback(self, msg: Int32MultiArray):
        """Callback for incoming servo commands."""
        if len(msg.data) != 3:
            self.get_logger().error("Invalid message format. Expected [servo_id, position, time_ms]")
            return

        servo_id, position, time_ms = msg.data
        self.move_servo(servo_id, position, time_ms)

    def move_servo(self, servo_id: int, position: int, time_ms: int):
        """Send a move command to the LSC-32."""
        # Validate ranges
        if not (0 <= servo_id <= 31):
            self.get_logger().error("Servo ID must be 0-31")
            return
        if not (0 <= position <= 1000):
            self.get_logger().error("Position must be 0-1000")
            return
        if not ( 0<= time_ms <= 30000):
            self.get_logger().error("Time must be >= 0 , <= 30000")
            return

        # Build packet according to LSC-32 protocol
        cmd = 3  # Move command
        num_servos = 1
        length = 5 + num_servos * 3
        packet = bytearray()
        packet.extend(b'\x55\x55')  # Header
        packet.append(length)
        packet.append(cmd)
        packet.extend(struct.pack('<H', time_ms))  # Little-endian time
        packet.append(num_servos)
        packet.append(servo_id)
        packet.extend(struct.pack('<H', position))  # Little-endian position

        try:
            self.ser.write(packet)
            self.get_logger().info(f"Sent: ID={servo_id}, Pos={position}, Time={time_ms}ms")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LSC32TopicController(port='/dev/ttyTHS1', baudrate=9600)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
