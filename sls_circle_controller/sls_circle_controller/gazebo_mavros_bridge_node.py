"""Gazebo 和 MAVROS 风格话题之间的轻量桥接节点。"""

import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped, Wrench
from mavros_msgs.msg import AttitudeTarget, State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter

from .controller_math import body_z_from_quat, clamp


class RateMeter:
    """按短时间窗口估计话题或回调的实时频率。"""

    def __init__(self):
        self.window_started = False
        self.window_start = 0.0
        self.sample_count = 0
        self.hz = 0.0

    def tick(self, now):
        if not self.window_started:
            self.window_start = now
            self.window_started = True
            self.sample_count = 0
        self.sample_count += 1
        elapsed = now - self.window_start
        if elapsed >= 0.5:
            self.hz = float(self.sample_count) / elapsed
            self.window_start = now
            self.sample_count = 0


class GazeboMavrosBridge(Node):
    """把 Gazebo 的 Odometry/force 插件包装成控制器可用的 MAVROS 接口。"""

    def __init__(self):
        super().__init__('gazebo_mavros_bridge')
        self._declare_parameters()
        self._read_parameters()
        self._apply_cli_parameter_overrides()
        self._normalize_parameters()
        self.last_wind_debug_log_time = 0.0
        self.odom_rate = RateMeter()
        self.load_odom_rate = RateMeter()
        self.force_rate = RateMeter()
        self.state_pub_rate = RateMeter()
        self.current_orientation = None
        self.current_angular_velocity = [0.0, 0.0, 0.0]
        self.commanded_wind = None

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 20)
        self.load_odom_sub = self.create_subscription(
            Odometry, self.load_odom_topic, self.load_odom_callback, 20)
        self.attitude_sub = self.create_subscription(
            AttitudeTarget, self.attitude_target_topic,
            self.attitude_callback, 20)
        self.wind_command_sub = self.create_subscription(
            Vector3Stamped, self.wind_command_topic,
            self.wind_command_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 20)
        self.velocity_pub = self.create_publisher(
            TwistStamped, self.velocity_topic, 20)
        self.load_pose_pub = self.create_publisher(
            PoseStamped, self.load_pose_topic, 20)
        self.state_pub = self.create_publisher(State, self.state_topic, 10)
        self.force_pub = self.create_publisher(Wrench, self.force_topic, 20)
        self.wind_actual_pub = self.create_publisher(
            Vector3Stamped, self.wind_actual_topic, 10)

        period = 1.0 / max(1.0, self.state_rate)
        self.state_timer = self.create_timer(period, self.publish_state)
        self.get_logger().info(
            'gazebo_mavros_bridge started; odom=%s force=%s'
            % (self.odom_topic, self.force_topic))

    def _declare_parameters(self):
        defaults = {
            'odom_topic': '/sls_circle/gazebo_odom',
            'load_odom_topic': '/sls_circle/gazebo_load_odom',
            'pose_topic': '/mavros/local_position/pose',
            'velocity_topic': '/mavros/local_position/velocity_local',
            'load_pose_topic': '/sls_circle/load_pose',
            'state_topic': '/mavros/state',
            'attitude_target_topic': '/sls_circle/debug/attitude_target',
            'force_topic': '/sls_circle/gazebo_force',
            'wind_actual_topic': '/sls_circle/wind_actual',
            'wind_command_topic': '/sls_circle/wind_command',
            'frame_id': 'map',
            'mass': 1.5,
            'gravity': 9.80665,
            'hover_thrust': 0.5,
            'max_force': 60.0,
            'attitude_kp': 1.2,
            'attitude_kd': 0.30,
            'max_torque': 1.0,
            'state_rate': 100.0,
            'enable_wind': False,
            'wind_force_x': 0.0,
            'wind_force_y': 0.0,
            'wind_force_z': 0.0,
            'wind_turbulence': 0.0,
            'wind_change_rate': 0.2,
            'wind_command_force_limit': 5.0,
            'enable_wind_debug_log': True,
            'wind_debug_log_period': 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        for name in [
            'odom_topic', 'load_odom_topic', 'pose_topic', 'velocity_topic',
            'load_pose_topic', 'state_topic', 'attitude_target_topic', 'force_topic',
            'wind_actual_topic', 'wind_command_topic', 'frame_id', 'mass', 'gravity',
            'hover_thrust', 'max_force', 'attitude_kp', 'attitude_kd',
            'max_torque', 'state_rate', 'enable_wind',
            'wind_force_x', 'wind_force_y', 'wind_force_z',
            'wind_turbulence', 'wind_change_rate', 'wind_command_force_limit',
            'enable_wind_debug_log', 'wind_debug_log_period',
        ]:
            setattr(self, name, self.get_parameter(name).value)
        self._normalize_parameters()

    def _normalize_parameters(self):
        self.mass = max(0.05, float(self.mass))
        self.state_rate = max(1.0, float(self.state_rate))
        self.hover_thrust = max(0.05, float(self.hover_thrust))
        self.max_force = max(0.1, float(self.max_force))
        self.attitude_kp = max(0.0, float(self.attitude_kp))
        self.attitude_kd = max(0.0, float(self.attitude_kd))
        self.max_torque = max(0.0, float(self.max_torque))
        self.wind_change_rate = max(0.0, float(self.wind_change_rate))
        self.wind_command_force_limit = max(
            0.0, float(self.wind_command_force_limit))
        self.wind_debug_log_period = max(0.1, float(self.wind_debug_log_period))
        self.enable_wind_debug_log = self.as_bool(self.enable_wind_debug_log)
        self.enable_wind = self.as_bool(self.enable_wind)

    def _apply_cli_parameter_overrides(self):
        # Foxy 的 Python launch 在某些场景下参数覆盖会变成 CLI token，这里手动兜底解析。
        float_names = {
            'mass', 'max_force', 'attitude_kp', 'attitude_kd', 'max_torque',
            'state_rate', 'wind_force_x', 'wind_force_y',
            'wind_force_z', 'wind_turbulence', 'wind_change_rate',
            'wind_debug_log_period',
            'wind_command_force_limit',
        }
        bool_names = {'enable_wind', 'enable_wind_debug_log'}
        args = sys.argv[1:]
        index = 0
        while index < len(args):
            arg = args[index]
            token = None
            if arg in ('-p', '--param') and index + 1 < len(args):
                token = args[index + 1]
                index += 2
            else:
                index += 1
            if token is None or ':=' not in token:
                continue

            name, value = token.split(':=', 1)
            if '.' in name:
                name = name.rsplit('.', 1)[-1]
            try:
                if name in float_names:
                    parsed = float(value)
                    setattr(self, name, parsed)
                    self.set_parameters([Parameter(name, value=parsed)])
                elif name in bool_names:
                    parsed = self.as_bool(value)
                    setattr(self, name, parsed)
                    self.set_parameters([Parameter(name, value=parsed)])
            except Exception as exc:
                self.get_logger().warn(
                    'ignored CLI parameter override %s:=%s: %s'
                    % (name, value, exc))

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def odom_callback(self, msg):
        # Gazebo p3d 输出的机体 odom 被转发成 MAVROS local_position 风格位姿和速度。
        self.odom_rate.tick(self.now_seconds())
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.pose_pub.publish(pose)
        self.current_orientation = msg.pose.pose.orientation
        velocity = TwistStamped()
        velocity.header.stamp = msg.header.stamp
        velocity.header.frame_id = self.frame_id
        velocity.twist = msg.twist.twist
        velocity.twist.linear = self.rotate_body_to_world(
            msg.twist.twist.linear, msg.pose.pose.orientation)
        angular_world = self.rotate_body_to_world(
            msg.twist.twist.angular, msg.pose.pose.orientation)
        self.current_angular_velocity = [
            angular_world.x, angular_world.y, angular_world.z]
        self.velocity_pub.publish(velocity)

    @staticmethod
    def rotate_body_to_world(vector, quat):
        qx = quat.x
        qy = quat.y
        qz = quat.z
        qw = quat.w
        vx = vector.x
        vy = vector.y
        vz = vector.z
        # v_world = q * v_body * q^-1，适配 Gazebo odom 中以 child frame 表达的 twist。
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        out = Vector3()
        out.x = vx + qw * tx + (qy * tz - qz * ty)
        out.y = vy + qw * ty + (qz * tx - qx * tz)
        out.z = vz + qw * tz + (qx * ty - qy * tx)
        return out

    def load_odom_callback(self, msg):
        self.load_odom_rate.tick(self.now_seconds())
        pose = PoseStamped()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        self.load_pose_pub.publish(pose)

    def attitude_callback(self, msg):
        # 控制器输出姿态目标，桥接节点把机体 z 轴方向和归一化推力换算成世界系力。
        self.force_rate.tick(self.now_seconds())
        q = msg.orientation
        body_z = body_z_from_quat(q.x, q.y, q.z, q.w)
        force_norm = (
            self.mass * float(self.gravity) * float(msg.thrust)
            / self.hover_thrust
        )
        force_norm = clamp(force_norm, 0.0, self.max_force)

        wrench = Wrench()
        wrench.force.x = body_z[0] * force_norm
        wrench.force.y = body_z[1] * force_norm
        wrench.force.z = body_z[2] * force_norm
        wind = self.current_wind_force()
        wrench.force.x += wind[0]
        wrench.force.y += wind[1]
        wrench.force.z += wind[2]
        torque = self.attitude_torque(q)
        wrench.torque.x = torque[0]
        wrench.torque.y = torque[1]
        wrench.torque.z = torque[2]
        self.publish_wind(wind)
        self.maybe_log_wind_debug(wind)
        self.force_pub.publish(wrench)

    def attitude_torque(self, desired):
        """World-frame quaternion PD torque that represents the missing PX4 attitude loop."""
        current = self.current_orientation
        if current is None:
            return [0.0, 0.0, 0.0]
        # q_error = q_desired * conjugate(q_current), shortest rotation.
        dx, dy, dz, dw = desired.x, desired.y, desired.z, desired.w
        cx, cy, cz, cw = -current.x, -current.y, -current.z, current.w
        ex = dw * cx + dx * cw + dy * cz - dz * cy
        ey = dw * cy - dx * cz + dy * cw + dz * cx
        ez = dw * cz + dx * cy - dy * cx + dz * cw
        ew = dw * cw - dx * cx - dy * cy - dz * cz
        sign = 1.0 if ew >= 0.0 else -1.0
        rotation_error = [2.0 * sign * ex, 2.0 * sign * ey, 2.0 * sign * ez]
        torque = [
            self.attitude_kp * rotation_error[i]
            - self.attitude_kd * self.current_angular_velocity[i]
            for i in range(3)
        ]
        magnitude = math.sqrt(sum(value * value for value in torque))
        if self.max_torque > 0.0 and magnitude > self.max_torque:
            scale = self.max_torque / magnitude
            torque = [value * scale for value in torque]
        return torque

    def current_wind_force(self):
        # 简化风模型只在水平面为主要验证对象，z 轴风力保持配置值，默认应为 0。
        if self.commanded_wind is not None:
            return list(self.commanded_wind)
        if not self.enable_wind:
            return [0.0, 0.0, 0.0]
        t = self.now_seconds()
        noise = float(self.wind_turbulence)
        rate = float(self.wind_change_rate)
        return [
            float(self.wind_force_x) + noise * math.sin(rate * t),
            float(self.wind_force_y) + noise * math.sin(0.7 * rate * t + 1.3),
            0.0,
        ]

    def wind_command_callback(self, msg):
        """接收 GUI 的水平风力指令，并在桥接层覆盖静态风参数。"""
        x = float(msg.vector.x)
        y = float(msg.vector.y)
        if not (math.isfinite(x) and math.isfinite(y)):
            self.get_logger().warn('ignored non-finite wind command')
            return
        magnitude = math.hypot(x, y)
        limit = self.wind_command_force_limit
        if limit > 0.0 and magnitude > limit:
            scale = limit / magnitude
            x *= scale
            y *= scale
        self.commanded_wind = [x, y, 0.0]

    def publish_wind(self, wind):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.vector.x = wind[0]
        msg.vector.y = wind[1]
        msg.vector.z = wind[2]
        self.wind_actual_pub.publish(msg)

    def maybe_log_wind_debug(self, wind):
        if not self.enable_wind_debug_log:
            return
        now = self.now_seconds()
        if now - self.last_wind_debug_log_time < self.wind_debug_log_period:
            return
        self.last_wind_debug_log_time = now
        mag = math.sqrt(wind[0] * wind[0] + wind[1] * wind[1] + wind[2] * wind[2])
        if mag > 1.0e-9:
            direction = [wind[0] / mag, wind[1] / mag, wind[2] / mag]
        else:
            direction = [0.0, 0.0, 0.0]
        self.get_logger().info(
            'actual applied wind force_n=(%.3f, %.3f, %.3f) '
            'mag_n=%.3f dir=(%.3f, %.3f, %.3f); '
            'rt_hz force=%.1f odom=%.1f load_odom=%.1f state=%.1f target_state=%.1f'
            % (
                wind[0], wind[1], wind[2],
                mag, direction[0], direction[1], direction[2],
                self.force_rate.hz, self.odom_rate.hz, self.load_odom_rate.hz,
                self.state_pub_rate.hz, float(self.state_rate),
            ))

    def publish_state(self):
        self.state_pub_rate.tick(self.now_seconds())
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        msg.armed = True
        msg.guided = True
        msg.manual_input = False
        msg.mode = 'OFFBOARD'
        msg.system_status = 4
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboMavrosBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
