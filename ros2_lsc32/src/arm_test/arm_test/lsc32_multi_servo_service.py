#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
import sys

from arm_servie.srv import MultiServo  # Generated from srv/MultiServo.srv

class LSC32MultiServoService(Node):
    """
    ROS2 node that exposes a service to send multi-servo commands to LSC-32.
    """

    def __init__(self):
        super().__init__('lsc32_multi_servo_service')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Connect to LSC-32
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(f"Connected to LSC-32 on {port} at {baudrate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            sys.exit(1)

        # Create service
        self.srv = self.create_service(MultiServo, 'move_servos', self.handle_move_servos)

    def handle_move_servos(self, request, response):
        """Handle incoming service request."""
        ids = request.ids
        positions = request.positions
        time_ms = request.time_ms

        # Validate
        if len(ids) != len(positions):
            response.success = False
            response.message = "IDs and positions arrays must have the same length"
            return response

        for sid in ids:
            if not (0 <= sid <= 31):
                response.success = False
                response.message = f"Invalid Servo ID {sid} (must be 031)"
                return response

        for pos in positions:
            if not (0 <= pos <= 1000):
                response.success = False
                response.message = f"Invalid Position {pos} (must be 01000)"
                return response

        if time_ms < 0:
            response.success = False
            response.message = "Time must be >= 0"
            return response

        # Build packet
        num_servos = len(ids)
        cmd = 3  # Move command
        length = 5 + num_servos * 3

        packet = bytearray()
        packet.extend(b'\x55\x55')  # Header
        packet.append(length)
        packet.append(cmd)
        packet.extend(struct.pack('<H', time_ms))
        packet.append(num_servos)

        for sid, pos in zip(ids, positions):
            packet.append(sid)
            packet.extend(struct.pack('<H', pos))

        try:
            self.ser.write(packet)
            response.success = True
            response.message = f"Sent {num_servos} servo commands in {time_ms}ms"
            self.get_logger().info(response.message)
        except serial.SerialException as e:
            response.success = False
            response.message = f"Failed to send command: {e}"

        return response

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LSC32MultiServoService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
