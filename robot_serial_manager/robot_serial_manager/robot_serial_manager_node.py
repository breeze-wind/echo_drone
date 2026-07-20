#!/usr/bin/env python3
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

import serial.tools.list_ports


class RobotSerialManager(Node):
    def __init__(self):
        super().__init__('robot_serial_manager')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('scan_period', 1.0)
        self.declare_parameter('devices', ['px4_fcu', 'stm32_servo', 'openmv'])

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.scan_period = float(self.get_parameter('scan_period').value)
        self.device_names = list(self.get_parameter('devices').value)
        self.devices = {}

        for name in self.device_names:
            self.devices[name] = self._declare_device(name)

        self.status_pub = self.create_publisher(String, '/hardware/serial_status', 10)
        self.rescan_srv = self.create_service(
            Trigger, '/hardware/rescan_serials', self.rescan_callback)
        self.scan_timer = self.create_timer(self.scan_period, self.scan_and_publish)
        self.latest_status = {}

        self.get_logger().info(
            'Serial manager started: dry_run=%s devices=%s'
            % (self.dry_run, ','.join(self.device_names))
        )
        self.scan_and_publish()

    def _declare_device(self, name):
        prefix = 'device.%s' % name
        self.declare_parameter(prefix + '.required', False)
        self.declare_parameter(prefix + '.owner', '')
        self.declare_parameter(prefix + '.port', '')
        self.declare_parameter(prefix + '.baudrate', 0)
        self.declare_parameter(prefix + '.manufacturer', '')

        return {
            'name': name,
            'required': bool(self.get_parameter(prefix + '.required').value),
            'owner': str(self.get_parameter(prefix + '.owner').value),
            'port': str(self.get_parameter(prefix + '.port').value),
            'baudrate': int(self.get_parameter(prefix + '.baudrate').value),
            'manufacturer': str(self.get_parameter(prefix + '.manufacturer').value),
        }

    def rescan_callback(self, request, response):
        del request
        self.scan_and_publish()
        response.success = True
        response.message = json.dumps(self.latest_status, ensure_ascii=False)
        return response

    def scan_and_publish(self):
        ports = list(serial.tools.list_ports.comports())
        status = {
            'stamp': time.time(),
            'dry_run': self.dry_run,
            'devices': [],
        }

        for device in self.devices.values():
            status['devices'].append(self._status_for_device(device, ports))

        self.latest_status = status
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _status_for_device(self, device, ports):
        configured_port = device['port']
        configured_exists = bool(configured_port) and os.path.exists(configured_port)
        matched_port = None

        if configured_exists:
            matched_port = self._port_by_path(configured_port, ports)
        elif device['manufacturer']:
            matched_port = self._port_by_manufacturer(device['manufacturer'], ports)

        resolved_port = configured_port if configured_exists else ''
        fallback_used = False
        if matched_port is not None:
            resolved_port = matched_port.device
            fallback_used = not configured_exists

        if resolved_port:
            state = 'available'
        elif self.dry_run:
            state = 'dry_run_missing'
        elif device['required']:
            state = 'missing_required'
        else:
            state = 'missing_optional'

        return {
            'name': device['name'],
            'owner': device['owner'],
            'required': device['required'],
            'configured_port': configured_port,
            'configured_exists': configured_exists,
            'resolved_port': resolved_port,
            'fallback_used': fallback_used,
            'baudrate': device['baudrate'],
            'manufacturer': device['manufacturer'],
            'state': state,
            'port_info': self._port_info(matched_port),
        }

    @staticmethod
    def _port_by_path(path, ports):
        for port in ports:
            if port.device == path:
                return port
        return None

    @staticmethod
    def _port_by_manufacturer(manufacturer, ports):
        for port in ports:
            if port.manufacturer == manufacturer:
                return port
        return None

    @staticmethod
    def _port_info(port):
        if port is None:
            return {}
        return {
            'device': port.device,
            'name': port.name,
            'description': port.description,
            'manufacturer': port.manufacturer,
            'hwid': port.hwid,
        }


def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
