#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import sys
import struct
from arm_servie.srv import MultiServo  # Generated from srv file

class LSC32MultiServiceNode(Node):
    """
    ROS2 Service Node to control multiple Hiwonder LSC32 servos in one command.
    """

    def __init__(self, port='/dev/ttyTHS1', baudrate=9600):
        super().__init__('lsc32_multi_service_node')

        # Try to open serial connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to LSC32 on {port} at {baudrate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)

        # Create ROS2 service
        self.srv = self.create_service(MultiServo, 'move_servos', self.handle_move_servos)

    def checksum(self, data_bytes):
        """Calculate checksum for LSC32 packet."""
        return (~sum(data_bytes) & 0xFF)

    def send_multi_move_command(self, ids, positions, time_ms):
        """Send multi-servo move command to LSC32."""
        if len(ids) != len(positions):
            return False, "IDs and positions arrays must have the same length"
        if not ids:
            return False, "No servos specified"
        if time_ms < 0:
            return False, "Time must be non-negative"

        # Validate each servo
        for sid, pos in zip(ids, positions):
            if not (0 <= sid <= 31):
                return False, f"Invalid servo ID: {sid}"
            if not (0 <= pos <= 1000):
                return False, f"Invalid position for servo {sid}: {pos}"

        # LSC32 multi-servo protocol:
        # 0x55 0x55 LEN CMD NUM_SERVOS TIME_L TIME_H [ID POS_L POS_H]... CHK
        cmd = 0x03  # Move servos
        num_servos = len(ids)
        time_l = time_ms & 0xFF
        time_h = (time_ms >> 8) & 0xFF

        # Build packet
        packet = [0x55, 0x55]
        length = 5 + num_servos * 3  # LEN = CMD + NUM + TIME_L + TIME_H + (3 bytes per servo)
        packet.append(length)
        packet.append(cmd)
        packet.append(num_servos)
        # packet.extend(struct.pack('<H', time_ms))  # Little-endian time        
        packet.append(time_l)
        packet.append(time_h)

        for sid, pos in zip(ids, positions):
           # packet.append(sid)
           # packet.extend(struct.pack('<H', pos))  # Little-endian position
           pos_l = pos & 0xFF
           pos_h = (pos >> 8) & 0xFF
           packet.extend([sid, pos_l, pos_h])

        # packet.append(self.checksum(packet[2:]))

        try:
            self.ser.write(bytearray(packet))
            return True, f"Sent multi-servo command: {list(zip(ids, positions))} in {time_ms}ms"
        except serial.SerialException as e:
            return False, f"Serial write error: {e}"

    def handle_move_servos(self, request, response):
        """Service callback."""
        success, message = self.send_multi_move_command(request.ids, request.positions, request.time_ms)
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return response

    def destroy_node(self):
        """Close serial port on shutdown."""
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LSC32MultiServiceNode(port='/dev/ttyTHS1', baudrate=9600)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

