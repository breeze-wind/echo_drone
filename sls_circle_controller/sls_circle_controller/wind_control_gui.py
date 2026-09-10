"""Gazebo 水平风力实时调节 GUI。"""

import math
import tkinter as tk
from tkinter import ttk

import rclpy
from geometry_msgs.msg import Vector3Stamped, Wrench
from rclpy.node import Node


class WindCommandNode(Node):
    def __init__(self):
        super().__init__('wind_control_gui')
        self.declare_parameter('wind_command_topic', '/sls_circle/wind_command')
        topic = self.get_parameter('wind_command_topic').value
        self.publisher = self.create_publisher(Vector3Stamped, topic, 10)
        self.declare_parameter(
            'px4_wind_force_topic', '/sls_circle/px4_wind_force')
        px4_topic = self.get_parameter('px4_wind_force_topic').value
        self.px4_publisher = self.create_publisher(Wrench, px4_topic, 10)
        self.get_logger().info('wind GUI publishing to %s' % topic)
        self.get_logger().info('PX4 Gazebo wind publishing to %s' % px4_topic)

    def publish_wind(self, x, y):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.vector.x = float(x)
        msg.vector.y = float(y)
        msg.vector.z = 0.0
        self.publisher.publish(msg)
        wrench = Wrench()
        wrench.force.x = float(x)
        wrench.force.y = float(y)
        wrench.force.z = 0.0
        self.px4_publisher.publish(wrench)


class WindControlGui:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Echo Drone - Gazebo 风力控制')
        self.root.resizable(False, False)
        self.enabled = tk.BooleanVar(value=False)
        self.magnitude = tk.DoubleVar(value=0.0)
        self.direction = tk.DoubleVar(value=0.0)
        self.vector_text = tk.StringVar(value='Fx = 0.00 N    Fy = 0.00 N')
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.after(100, self.update)

    def _build(self):
        frame = ttk.Frame(self.root, padding=14)
        frame.grid()
        ttk.Label(frame, text='风力大小（N）').grid(row=0, column=0, sticky='w')
        tk.Scale(
            frame, from_=0.0, to=3.0, resolution=0.01,
            orient='horizontal', length=360, variable=self.magnitude,
            command=self.refresh_label,
        ).grid(row=1, column=0, columnspan=3)
        ttk.Label(frame, text='方向（度）  0°=+X，90°=+Y').grid(
            row=2, column=0, sticky='w', pady=(10, 0))
        tk.Scale(
            frame, from_=-180, to=180, resolution=1,
            orient='horizontal', length=360, variable=self.direction,
            command=self.refresh_label,
        ).grid(row=3, column=0, columnspan=3)
        ttk.Checkbutton(
            frame, text='启用风力', variable=self.enabled,
            command=self.refresh_label,
        ).grid(row=4, column=0, sticky='w', pady=(10, 0))
        ttk.Button(frame, text='归零', command=self.zero).grid(
            row=4, column=1, padx=8, pady=(10, 0))
        ttk.Button(frame, text='关闭', command=self.close).grid(
            row=4, column=2, pady=(10, 0))
        ttk.Label(frame, textvariable=self.vector_text).grid(
            row=5, column=0, columnspan=3, sticky='w', pady=(12, 0))
        ttk.Label(
            frame, text='建议首次从 0.05 N 开始，慢慢增加。',
            foreground='#8a3b12',
        ).grid(row=6, column=0, columnspan=3, sticky='w', pady=(8, 0))

    def components(self):
        if not self.enabled.get():
            return 0.0, 0.0
        angle = math.radians(self.direction.get())
        magnitude = self.magnitude.get()
        return magnitude * math.cos(angle), magnitude * math.sin(angle)

    def refresh_label(self, *_):
        x, y = self.components()
        self.vector_text.set('Fx = %.2f N    Fy = %.2f N' % (x, y))

    def zero(self):
        self.enabled.set(False)
        self.magnitude.set(0.0)
        self.refresh_label()
        self.node.publish_wind(0.0, 0.0)

    def update(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        self.node.publish_wind(*self.components())
        self.refresh_label()
        self.root.after(100, self.update)

    def close(self):
        self.node.publish_wind(0.0, 0.0)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = WindCommandNode()
    try:
        WindControlGui(node).run()
    except tk.TclError as exc:
        node.get_logger().error('cannot open wind GUI: %s' % exc)
    except KeyboardInterrupt:
        node.publish_wind(0.0, 0.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
