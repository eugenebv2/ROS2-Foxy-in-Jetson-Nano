#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import struct
import sys

class LSC32MultiServoController(Node):
    """
    ROS2 node that listens to /servo_cmd topic and sends multi-servo commands to LSC-32.
    Message format: Int32MultiArray with [id1, pos1, id2, pos2, ..., time_ms]
    Example: data=[1, 500, 2, 600, 1000] -> Move servo 1 to 500, servo 2 to 600 in 1000ms
    """

    def __init__(self, port='/dev/ttyTHS1', baudrate=9600):
        super().__init__('lsc32_multi_servo_controller')

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
        """Callback for incoming multi-servo commands."""
        if len(msg.data) < 3 or len(msg.data) % 2 != 1:
            self.get_logger().error(
                "Invalid message format. Expected [id1, pos1, id2, pos2, ..., time_ms]"
            )
            return

        time_ms = msg.data[-1]
        servo_pairs = msg.data[:-1]

        # Build list of (id, pos) tuples
        servos = []
        for i in range(0, len(servo_pairs), 2):
            servos.append((servo_pairs[i], servo_pairs[i + 1]))

        self.move_servos(servos, time_ms)

    def move_servos(self, servos, time_ms: int):
        """Send a multi-servo move command to the LSC-32."""
        # Validate ranges
        for sid, pos in servos:
            if not (0 <= sid <= 31):
                self.get_logger().error(f"Invalid Servo ID {sid} (must be 031)")
                return
            if not (0 <= pos <= 1000):
                self.get_logger().error(f"Invalid Position {pos} (must be 01000)")
                return
        if time_ms < 0:
            self.get_logger().error("Time must be >= 0")
            return

        num_servos = len(servos)
        cmd = 3  # Move command
        length = 5 + num_servos * 3  # LEN = params + 3

        # Build packet
        packet = bytearray()
        packet.extend(b'\x55\x55')  # Header
        packet.append(length)
        packet.append(cmd)
        packet.extend(struct.pack('<H', time_ms))  # Little-endian time
        packet.append(num_servos)

        for sid, pos in servos:
            packet.append(sid)
            packet.extend(struct.pack('<H', pos))  # Little-endian position

        try:
            self.ser.write(packet)
            self.get_logger().info(
                f"Sent multi-servo command: {servos} in {time_ms}ms"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LSC32MultiServoController(port='/dev/ttyTHS1', baudrate=9600)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
