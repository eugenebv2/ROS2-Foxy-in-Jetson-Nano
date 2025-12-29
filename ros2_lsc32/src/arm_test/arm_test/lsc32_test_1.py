#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import UInt16MultiArray

class LSC32Controller(Node):
    def __init__(self):
        super().__init__('lsc32_controller')

        # Parameters
        #self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('port', '/dev/ttyTHS1')
        #self.declare_parameter('baudrate', 115200)
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Connected to LSC32 on {port} at {baudrate} bps")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection failed: {e}")
            raise SystemExit

        # Subscribe to servo commands: [id1, pos1, id2, pos2, ..., time_ms]
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            'lsc32_cmd',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg: UInt16MultiArray):
        data = msg.data
        if len(data) < 3 or len(data) % 2 != 1:
            self.get_logger().error("Invalid command format. Expected: [id, pos, ..., time_ms]")
            return

        time_ms = data[-1]
        servo_data = data[:-1]
        num_servos = len(servo_data) // 2

        # Build packet
        packet = bytearray()
        packet.extend([0x55, 0x55])
        length = 3 + num_servos * 3  # CMD + count + time(2B) + servo data
        packet.append(length)
        packet.append(0x03)  # CMD: Move servos
        packet.append(num_servos)
        packet.append(time_ms & 0xFF)
        packet.append((time_ms >> 8) & 0xFF)

        for i in range(0, len(servo_data), 2):
            sid = servo_data[i]
            pos = servo_data[i + 1]
            packet.append(sid)
            packet.append(pos & 0xFF)
            packet.append((pos >> 8) & 0xFF)

        # Checksum
        checksum = sum(packet[2:]) & 0xFF
        packet.append(checksum)

        try:
            self.ser.write(packet)
            self.get_logger().info(f"Sent: {list(packet)}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LSC32Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

