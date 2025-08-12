#!/usr/bin/env python3
import serial
import serial.tools.list_ports
import rclpy
import struct
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.get_logger().info('Servo Node Launch')
        self.declare_parameter('/servo/servo', 0)
        self.timer = self.create_timer(0.1, self.timer_callback)

        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.manufacturer == 'STMicroelectronics':
                self.serial_port = serial.Serial(#what the fuck
                    port=port.device,
                    baudrate=115200,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                )

    def timer_callback(self):
        # 查询参数值
        servo_param = self.get_parameter('/servo/servo')

        servo_cmd = servo_param.get_parameter_value().integer_value
        self.get_logger().info(f'Parameter value: {servo_cmd}')
        serial_buff = struct.pack('B', servo_cmd)
        self.serial_port.write(serial_buff)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
