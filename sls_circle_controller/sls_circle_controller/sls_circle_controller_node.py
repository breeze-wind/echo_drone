"""ROS2 MAVROS 圆周控制器的早期 Python 实现。

当前实机和 PX4 SITL 主路径已经迁到 C++ 节点，本文件保留用于算法对照和快速阅读。
"""

import json
import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import AttitudeTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rclpy.node import Node
from std_msgs.msg import String

from .controller_math import (
    LesoAxis,
    attitude_from_accel,
    clamp,
    cross,
    norm3,
    normalize3,
    unit_vector,
)
from .drown_qsf import QsfCore, QsfCoreError


@dataclass
class Reference:
    """控制器内部统一参考轨迹，QSF 会用到高阶导数。"""

    pos: list
    vel: list
    acc: list
    jerk: list
    snap: list


class SlsCircleController(Node):
    """早期 Python 版 SLS/QSF 控制器，接口与 C++ 主控制器基本一致。"""

    def __init__(self):
        super().__init__('sls_circle_controller')

        self._declare_parameters()
        self._read_parameters()

        self.pose = None
        self.pose_time = None
        self.velocity = [0.0, 0.0, 0.0]
        self.velocity_time = None
        self.observed_accel = [0.0, 0.0, 0.0]
        self.velocity_valid = False
        self.observed_accel_valid = False
        self.load_pose = None
        self.load_pose_time = None
        self.load_velocity = [0.0, 0.0, 0.0]
        self.load_observed_accel = [0.0, 0.0, 0.0]
        self.load_velocity_valid = False
        self.load_observed_accel_valid = False
        self.mavros_state = State()
        self.start_time = None
        self.circle_start_time = None
        self.center_locked = False
        self.center = [self.center_x, self.center_y, self.center_z]
        self.home_position = None
        self.last_control_time = None
        self.last_commanded_accel = [0.0, 0.0, 0.0]
        self.last_commanded_accel_valid = False
        self.flight_stage = (
            'preflight' if self.mission_mode == 'takeoff_then_circle'
            else 'circle')
        self.takeoff_reached_time = None
        self.last_mode_request_time = 0.0
        self.last_arm_request_time = 0.0
        self.disturbance = [0.0, 0.0, 0.0]
        self.wind_integral = [0.0, 0.0, 0.0]
        self.wind_compensation_accel = [0.0, 0.0, 0.0]
        self.wind_compensation_force = [0.0, 0.0, 0.0]
        self.wind_compensation_magnitude = 0.0
        self.wind_compensation_direction = [0.0, 0.0, 0.0]
        self.effective_wind_compensation_gain = 0.0
        self.actual_wind_force = [0.0, 0.0, 0.0]
        self.actual_wind_magnitude = 0.0
        self.actual_wind_direction = [0.0, 0.0, 0.0]
        self.actual_wind_time = None
        self.last_wind_debug_log_time = 0.0
        self.qsf_force_ned = [0.0, 0.0, 0.0]
        self.qsf_xi = [0.0, 0.0, 0.0]
        self.qsf_error = ''
        self.controller_source = 'startup'
        self.qsf_core = None
        self.qsf_core_version = ''

        self.leso = [
            LesoAxis(self.wind_observer_bandwidth),
            LesoAxis(self.wind_observer_bandwidth),
            LesoAxis(self.wind_observer_bandwidth),
        ]
        self._load_qsf_core()

        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_callback, 20)
        self.velocity_sub = self.create_subscription(
            TwistStamped, self.velocity_topic, self.velocity_callback, 20)
        self.load_pose_sub = self.create_subscription(
            PoseStamped, self.load_pose_topic, self.load_pose_callback, 20)
        self.state_sub = self.create_subscription(
            State, self.state_topic, self.state_callback, 20)
        self.actual_wind_sub = self.create_subscription(
            Vector3Stamped, self.actual_wind_topic, self.actual_wind_callback, 10)

        self.reference_pub = self.create_publisher(
            PoseStamped, self.reference_pose_topic, 10)
        self.debug_attitude_pub = self.create_publisher(
            AttitudeTarget, self.debug_attitude_topic, 10)
        self.real_attitude_pub = self.create_publisher(
            AttitudeTarget, self.real_attitude_topic, 10)
        self.takeoff_pose_pub = self.create_publisher(
            PoseStamped, self.takeoff_pose_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.wind_estimate_pub = self.create_publisher(
            Vector3Stamped, self.wind_estimate_topic, 10)

        self.set_mode_client = self.create_client(SetMode, self.set_mode_service)
        self.arming_client = self.create_client(CommandBool, self.arming_service)

        period = 1.0 / max(1.0, self.control_rate)
        self.timer = self.create_timer(period, self.control_loop)
        self.get_logger().info(
            'sls_circle_controller started; mode=%s dry_run=%s real_setpoint=%s'
            % (self.controller_mode, self.dry_run, self.enable_real_setpoint))

    def _declare_parameters(self):
        # 参数结构与 C++ 主控制器保持相近，便于对照行为差异。
        defaults = {
            'dry_run': True,
            'enable_real_setpoint': False,
            'pose_topic': '/mavros/local_position/pose',
            'velocity_topic': '/mavros/local_position/velocity_local',
            'use_velocity_topic': True,
            'velocity_stale_timeout': 0.5,
            'load_pose_topic': '/sls_circle/load_pose',
            'use_load_pose': False,
            'load_pose_stale_timeout': 0.5,
            'state_topic': '/mavros/state',
            'real_attitude_topic': '/mavros/setpoint_raw/attitude',
            'debug_attitude_topic': '/sls_circle/debug/attitude_target',
            'takeoff_pose_topic': '/mavros/setpoint_position/local',
            'reference_pose_topic': '/sls_circle/reference_pose',
            'status_topic': '/sls_circle/status',
            'wind_estimate_topic': '/sls_circle/wind_estimate',
            'actual_wind_topic': '/sls_circle/wind_actual',
            'frame_id': 'map',
            'control_rate': 50.0,
            'pose_stale_timeout': 0.5,
            'mission_mode': 'circle_only',
            'preflight_setpoint_time': 2.0,
            'takeoff_altitude': 1.0,
            'takeoff_x': 0.0,
            'takeoff_y': 0.0,
            'use_current_xy_for_takeoff': True,
            'takeoff_z_tolerance': 0.15,
            'takeoff_settle_time': 2.0,
            'enable_takeoff_position_setpoint': True,
            'enable_mavros_services': False,
            'auto_offboard': False,
            'auto_arm': False,
            'set_mode_service': '/mavros/set_mode',
            'arming_service': '/mavros/cmd/arming',
            'offboard_mode': 'OFFBOARD',
            'service_retry_period': 1.0,
            'use_current_pose_as_start': True,
            'center_x': 0.0,
            'center_y': 0.0,
            'center_z': 1.0,
            'center_z_from_pose': True,
            'radius': 1.0,
            'angular_velocity': 0.35,
            'circle_loops': 0.0,
            'phase': 0.0,
            'yaw': 0.0,
            'kp_xy': 1.8,
            'kp_z': 2.5,
            'kd_xy': 1.4,
            'kd_z': 1.6,
            'max_acc_xy': 2.5,
            'max_acc_z': 2.0,
            'max_total_acc': 4.0,
            'max_tilt_deg': 25.0,
            'gravity': 9.80665,
            'hover_thrust': 0.5,
            'min_thrust': 0.05,
            'max_thrust': 0.85,
            'require_connected': True,
            'require_offboard': False,
            'require_armed': False,
            'controller_mode': 'pd',
            'mav_mass': 1.56,
            'load_mass': 0.25,
            'cable_length': 0.85,
            'qsf_kp_x': 10.0,
            'qsf_kv_x': 5.0,
            'qsf_ka_x': 0.0,
            'qsf_kj_x': 0.0,
            'qsf_kp_y': 10.0,
            'qsf_kv_y': 5.0,
            'qsf_ka_y': 0.0,
            'qsf_kj_y': 0.0,
            'qsf_kp_z': 20.0,
            'qsf_kv_z': 10.0,
            'qsf_ki_x': 12.0,
            'qsf_ki_y': 12.0,
            'qsf_ki_z': 1.0,
            'qsf_integral_limit': 10.0,
            'qsf_reference_is_load': False,
            'enable_anti_wind': False,
            'wind_estimator_mode': 'residual',
            'wind_observer_bandwidth': 4.0,
            'wind_estimate_filter_tau': 0.5,
            'wind_estimate_force_limit': 5.0,
            'wind_compensation_gain': 0.7,
            'wind_compensation_warmup_time': 0.0,
            'wind_compensation_ramp_time': 0.0,
            'wind_integral_gain': 0.0,
            'wind_integral_limit': 5.0,
            'enable_wind_debug_log': True,
            'wind_debug_log_period': 1.0,
            'status_period': 0.25,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        names = [
            'dry_run', 'enable_real_setpoint', 'pose_topic', 'load_pose_topic',
            'velocity_topic', 'use_velocity_topic', 'velocity_stale_timeout',
            'use_load_pose', 'load_pose_stale_timeout', 'state_topic',
            'real_attitude_topic', 'debug_attitude_topic', 'takeoff_pose_topic',
            'reference_pose_topic', 'status_topic', 'wind_estimate_topic',
            'actual_wind_topic', 'frame_id', 'control_rate', 'pose_stale_timeout',
            'mission_mode',
            'preflight_setpoint_time', 'takeoff_altitude', 'takeoff_x',
            'takeoff_y', 'use_current_xy_for_takeoff', 'takeoff_z_tolerance',
            'takeoff_settle_time', 'enable_takeoff_position_setpoint',
            'enable_mavros_services', 'auto_offboard', 'auto_arm',
            'set_mode_service', 'arming_service', 'offboard_mode',
            'service_retry_period', 'use_current_pose_as_start', 'center_x',
            'center_y', 'center_z', 'center_z_from_pose', 'radius',
            'angular_velocity', 'circle_loops', 'phase', 'yaw', 'kp_xy',
            'kp_z', 'kd_xy', 'kd_z', 'max_acc_xy', 'max_acc_z',
            'max_total_acc', 'max_tilt_deg', 'gravity', 'hover_thrust',
            'min_thrust', 'max_thrust', 'require_connected',
            'require_offboard', 'require_armed', 'controller_mode',
            'mav_mass', 'load_mass', 'cable_length', 'qsf_kp_x', 'qsf_kv_x',
            'qsf_ka_x', 'qsf_kj_x', 'qsf_kp_y', 'qsf_kv_y', 'qsf_ka_y',
            'qsf_kj_y', 'qsf_kp_z', 'qsf_kv_z', 'qsf_ki_x', 'qsf_ki_y',
            'qsf_ki_z', 'qsf_integral_limit', 'qsf_reference_is_load',
            'enable_anti_wind', 'wind_estimator_mode', 'wind_observer_bandwidth',
            'wind_compensation_gain', 'wind_compensation_warmup_time',
            'wind_compensation_ramp_time', 'wind_integral_gain',
            'wind_integral_limit', 'wind_estimate_filter_tau',
            'wind_estimate_force_limit', 'enable_wind_debug_log',
            'wind_debug_log_period', 'status_period',
        ]
        for name in names:
            setattr(self, name, self.get_parameter(name).value)

        for name in [
            'dry_run', 'enable_real_setpoint', 'use_load_pose',
            'use_velocity_topic',
            'use_current_xy_for_takeoff', 'enable_takeoff_position_setpoint',
            'enable_mavros_services', 'auto_offboard', 'auto_arm',
            'use_current_pose_as_start', 'center_z_from_pose',
            'require_connected', 'require_offboard', 'require_armed',
            'qsf_reference_is_load', 'enable_anti_wind',
            'enable_wind_debug_log',
        ]:
            setattr(self, name, self.as_bool(getattr(self, name)))

        for name in [
            'load_pose_stale_timeout', 'velocity_stale_timeout',
            'control_rate', 'pose_stale_timeout',
            'preflight_setpoint_time', 'takeoff_altitude', 'takeoff_x',
            'takeoff_y', 'takeoff_z_tolerance', 'takeoff_settle_time',
            'service_retry_period', 'center_x', 'center_y', 'center_z',
            'radius', 'angular_velocity', 'circle_loops', 'phase', 'yaw',
            'kp_xy', 'kp_z', 'kd_xy', 'kd_z', 'max_acc_xy', 'max_acc_z',
            'max_total_acc', 'max_tilt_deg', 'gravity', 'hover_thrust',
            'min_thrust', 'max_thrust', 'mav_mass', 'load_mass',
            'cable_length', 'qsf_kp_x', 'qsf_kv_x', 'qsf_ka_x', 'qsf_kj_x',
            'qsf_kp_y', 'qsf_kv_y', 'qsf_ka_y', 'qsf_kj_y', 'qsf_kp_z',
            'qsf_kv_z', 'qsf_ki_x', 'qsf_ki_y', 'qsf_ki_z',
            'qsf_integral_limit', 'wind_observer_bandwidth',
            'wind_compensation_gain', 'wind_compensation_warmup_time',
            'wind_compensation_ramp_time', 'wind_integral_gain',
            'wind_integral_limit', 'wind_estimate_filter_tau',
            'wind_estimate_force_limit', 'wind_debug_log_period', 'status_period',
        ]:
            setattr(self, name, float(getattr(self, name)))

        self.mission_mode = str(self.mission_mode).lower()
        self.controller_mode = str(self.controller_mode).lower()
        self.wind_estimator_mode = str(self.wind_estimator_mode).lower()
        if self.wind_estimator_mode not in ('leso', 'actual_feedback'):
            self.wind_estimator_mode = 'residual'
        self.radius = max(0.05, float(self.radius))
        self.hover_thrust = max(0.05, float(self.hover_thrust))
        self.mav_mass = max(0.05, float(self.mav_mass))
        self.load_mass = max(0.0, float(self.load_mass))
        self.cable_length = max(0.05, float(self.cable_length))
        self.max_tilt_rad = math.radians(float(self.max_tilt_deg))
        self.preflight_setpoint_time = max(0.0, float(self.preflight_setpoint_time))
        self.takeoff_settle_time = max(0.0, float(self.takeoff_settle_time))
        self.velocity_stale_timeout = max(0.02, float(self.velocity_stale_timeout))
        self.wind_debug_log_period = max(0.1, float(self.wind_debug_log_period))
        self.wind_estimate_filter_tau = max(0.02, float(self.wind_estimate_filter_tau))
        self.wind_estimate_force_limit = max(0.0, float(self.wind_estimate_force_limit))
        self.wind_compensation_warmup_time = max(0.0, float(self.wind_compensation_warmup_time))
        self.wind_compensation_ramp_time = max(0.0, float(self.wind_compensation_ramp_time))

    @staticmethod
    def as_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        return str(value).strip().lower() in ('1', 'true', 'yes', 'on')

    def _load_qsf_core(self):
        if not self.controller_mode.startswith('qsf'):
            return
        try:
            self.qsf_core = QsfCore()
            self.qsf_core_version = self.qsf_core.version()
            self.get_logger().info('loaded QSF core: %s' % self.qsf_core_version)
        except (QsfCoreError, LookupError, OSError) as exc:
            self.qsf_core = None
            self.qsf_error = str(exc)
            self.get_logger().warning(
                'QSF core unavailable, falling back to PD: %s' % self.qsf_error)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def pose_callback(self, msg):
        now = self.now_seconds()
        new_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        velocity_fresh = (
            self.use_velocity_topic and hasattr(self, 'velocity_time')
            and self.velocity_time is not None
            and now - self.velocity_time <= self.velocity_stale_timeout
        )
        if self.pose is not None and self.pose_time is not None and not velocity_fresh:
            dt = max(1.0e-6, now - self.pose_time)
            new_velocity = [
                (new_pose[0] - self.pose[0]) / dt,
                (new_pose[1] - self.pose[1]) / dt,
                (new_pose[2] - self.pose[2]) / dt,
            ]
            if self.velocity_valid:
                self.observed_accel = [
                    (new_velocity[0] - self.velocity[0]) / dt,
                    (new_velocity[1] - self.velocity[1]) / dt,
                    (new_velocity[2] - self.velocity[2]) / dt,
                ]
                self.observed_accel_valid = True
            self.velocity = new_velocity
            self.velocity_valid = True
        self.pose = new_pose
        self.pose_time = now

        if self.home_position is None:
            self.home_position = list(self.pose)
        if self.start_time is None:
            self.start_time = now
            self.circle_start_time = now
            for axis, observer in enumerate(self.leso):
                observer.reset(self.pose[axis], self.velocity[axis])

    def velocity_callback(self, msg):
        if not self.use_velocity_topic:
            return
        now = self.now_seconds()
        new_velocity = [
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            float(msg.twist.linear.z),
        ]
        if hasattr(self, 'velocity_time') and self.velocity_time is not None:
            dt = max(1.0e-6, now - self.velocity_time)
            self.observed_accel = [
                (new_velocity[0] - self.velocity[0]) / dt,
                (new_velocity[1] - self.velocity[1]) / dt,
                (new_velocity[2] - self.velocity[2]) / dt,
            ]
            self.observed_accel_valid = True
        self.velocity = new_velocity
        self.velocity_time = now
        self.velocity_valid = True

    def load_pose_callback(self, msg):
        now = self.now_seconds()
        new_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        if self.load_pose is not None and self.load_pose_time is not None:
            dt = max(1.0e-6, now - self.load_pose_time)
            new_velocity = [
                (new_pose[0] - self.load_pose[0]) / dt,
                (new_pose[1] - self.load_pose[1]) / dt,
                (new_pose[2] - self.load_pose[2]) / dt,
            ]
            if self.load_velocity_valid:
                self.load_observed_accel = [
                    (new_velocity[0] - self.load_velocity[0]) / dt,
                    (new_velocity[1] - self.load_velocity[1]) / dt,
                    (new_velocity[2] - self.load_velocity[2]) / dt,
                ]
                self.load_observed_accel_valid = True
            self.load_velocity = new_velocity
            self.load_velocity_valid = True
        self.load_pose = new_pose
        self.load_pose_time = now

    def state_callback(self, msg):
        self.mavros_state = msg

    def actual_wind_callback(self, msg):
        self.actual_wind_force = [
            float(msg.vector.x), float(msg.vector.y), float(msg.vector.z)]
        self.actual_wind_magnitude = norm3(self.actual_wind_force)
        self.actual_wind_direction = unit_vector(self.actual_wind_force)
        self.actual_wind_time = self.now_seconds()

    def lock_center_from_pose(self):
        if self.center_locked or self.pose is None:
            return
        self.center = [self.center_x, self.center_y, self.center_z]
        if self.use_current_pose_as_start:
            self.center[0] = self.pose[0] - self.radius * math.cos(self.phase)
            self.center[1] = self.pose[1] - self.radius * math.sin(self.phase)
        if self.center_z_from_pose:
            self.center[2] = self.pose[2]
        self.center_locked = True
        self.get_logger().info(
            'circle center locked at x=%.3f y=%.3f z=%.3f'
            % (self.center[0], self.center[1], self.center[2]))

    def static_reference(self, x, y, z):
        zero = [0.0, 0.0, 0.0]
        return Reference([x, y, z], list(zero), list(zero), list(zero), list(zero))

    def circle_reference_at(self, elapsed):
        if self.circle_loops > 0.0 and self.angular_velocity > 1.0e-6:
            max_elapsed = self.circle_loops * 2.0 * math.pi / self.angular_velocity
            elapsed = min(elapsed, max_elapsed)

        theta = self.phase + self.angular_velocity * elapsed
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        w = self.angular_velocity
        r = self.radius
        pos = [self.center[0] + r * cos_t, self.center[1] + r * sin_t, self.center[2]]
        vel = [-r * w * sin_t, r * w * cos_t, 0.0]
        acc = [-r * w * w * cos_t, -r * w * w * sin_t, 0.0]
        jerk = [r * w * w * w * sin_t, -r * w * w * w * cos_t, 0.0]
        snap = [r * w * w * w * w * cos_t, r * w * w * w * w * sin_t, 0.0]
        return Reference(pos, vel, acc, jerk, snap)

    def update_flight_stage(self, now):
        if self.mission_mode != 'takeoff_then_circle':
            self.flight_stage = 'circle'
            if not self.center_locked:
                self.lock_center_from_pose()
            return

        if self.flight_stage == 'circle':
            return

        if self.start_time is None:
            self.flight_stage = 'preflight'
            return

        if now - self.start_time < self.preflight_setpoint_time:
            self.flight_stage = 'preflight'
            return

        self.maybe_request_offboard_and_arm(now)
        if self.pose is None:
            self.flight_stage = 'takeoff'
            return

        if self.pose[2] >= self.takeoff_altitude - self.takeoff_z_tolerance:
            if self.takeoff_reached_time is None:
                self.takeoff_reached_time = now
                self.flight_stage = 'settle'
                return
            if now - self.takeoff_reached_time >= self.takeoff_settle_time:
                self.enter_circle(now)
                return
            self.flight_stage = 'settle'
            return

        self.takeoff_reached_time = None
        self.flight_stage = 'takeoff'

    def enter_circle(self, now):
        self.flight_stage = 'circle'
        self.circle_start_time = now
        self.center_locked = False
        self.lock_center_from_pose()
        self.get_logger().info('takeoff confirmed; entering circle tracking')

    def takeoff_reference(self):
        if self.home_position is None:
            x = self.takeoff_x
            y = self.takeoff_y
        elif self.use_current_xy_for_takeoff:
            x = self.home_position[0]
            y = self.home_position[1]
        else:
            x = self.takeoff_x
            y = self.takeoff_y
        return self.static_reference(x, y, self.takeoff_altitude)

    def active_reference(self, now):
        if self.flight_stage != 'circle':
            return self.takeoff_reference()
        self.lock_center_from_pose()
        start = self.circle_start_time if self.circle_start_time is not None else self.start_time
        elapsed = max(0.0, now - start)
        return self.circle_reference_at(elapsed)

    def control_loop(self):
        # 主循环先检查位姿新鲜度，再计算参考轨迹、控制加速度和 MAVROS 姿态目标。
        now = self.now_seconds()
        if self.pose is None or self.pose_time is None:
            self.publish_status(now, 'waiting_for_pose')
            return

        pose_age = now - self.pose_time
        if pose_age > self.pose_stale_timeout:
            self.publish_status(now, 'pose_stale')
            return

        dt = 0.0
        if self.last_control_time is not None:
            dt = max(0.0, now - self.last_control_time)
        self.last_control_time = now

        self.update_flight_stage(now)
        reference = self.active_reference(now)

        accel = self.compute_accel(reference, dt)
        self.last_commanded_accel = list(accel)
        self.last_commanded_accel_valid = True
        attitude_msg = self.build_attitude_target(accel)
        reference_msg = self.build_reference_pose(reference.pos)

        self.reference_pub.publish(reference_msg)
        self.debug_attitude_pub.publish(attitude_msg)
        self.publish_takeoff_position(reference_msg)

        gate_ok, gate_reason = self.real_setpoint_gate()
        # 调试 setpoint 总是发布，真实 setpoint 只有通过 gate 后才发布。
        if gate_ok:
            self.real_attitude_pub.publish(attitude_msg)

        self.publish_status(
            now, gate_reason, ref=reference, accel=accel,
            real_active=gate_ok)

    def compute_accel(self, ref, dt):
        # 优先调用 QSF 动态库，失败时退回 PD，避免控制器完全停止输出。
        mode = self.controller_mode
        accel = None
        if mode.startswith('qsf') and self.qsf_core is not None:
            accel = self.compute_qsf_accel(ref, dt)

        if accel is None:
            accel = self.compute_pd_accel(ref)
            if mode.startswith('qsf') and self.qsf_core is None:
                self.controller_source = 'pd_fallback_qsf_unavailable'
            else:
                self.controller_source = 'pd'

        if self.enable_anti_wind or mode in ('leso_pd', 'leso'):
            accel = self.apply_anti_wind(ref, accel, dt)
        else:
            self.update_wind_compensation_debug([0.0, 0.0, 0.0])

        return self.limit_accel(accel)

    def compute_pd_accel(self, ref):
        err = [
            ref.pos[0] - self.pose[0],
            ref.pos[1] - self.pose[1],
            ref.pos[2] - self.pose[2],
        ]
        vel_err = [
            ref.vel[0] - self.velocity[0],
            ref.vel[1] - self.velocity[1],
            ref.vel[2] - self.velocity[2],
        ]
        return [
            ref.acc[0] + self.kp_xy * err[0] + self.kd_xy * vel_err[0],
            ref.acc[1] + self.kp_xy * err[1] + self.kd_xy * vel_err[1],
            ref.acc[2] + self.kp_z * err[2] + self.kd_z * vel_err[2],
        ]

    def compute_qsf_accel(self, ref, dt):
        try:
            # QSF 生成代码采用 north/east/down 语义，这里统一由 build_* 函数完成 ENU 映射。
            state = self.build_sls_state()
            params = [self.load_mass, self.mav_mass, self.cable_length, self.gravity]
            ref_x, ref_y, ref_z = self.build_qsf_refs(ref)
            if self.controller_mode == 'qsf_integral':
                qsf_state = list(state) + list(self.qsf_xi)
                gains = [
                    self.qsf_ki_x, self.qsf_kp_x, self.qsf_kv_x,
                    self.qsf_ka_x, self.qsf_kj_x, self.qsf_ki_y,
                    self.qsf_kp_y, self.qsf_kv_y, self.qsf_ka_y,
                    self.qsf_kj_y, self.qsf_ki_z, self.qsf_kp_z,
                    self.qsf_kv_z,
                ]
                force, xi_dot = self.qsf_core.qsf_integral(
                    qsf_state, gains, params, ref_x, ref_y, ref_z)
                self.integrate_qsf_xi(xi_dot, dt)
            else:
                gains = [
                    self.qsf_kp_x, self.qsf_kv_x, self.qsf_ka_x,
                    self.qsf_kj_x, self.qsf_kp_y, self.qsf_kv_y,
                    self.qsf_ka_y, self.qsf_kj_y, self.qsf_kp_z,
                    self.qsf_kv_z,
                ]
                force = self.qsf_core.qsf(state, gains, params, ref_x, ref_y, ref_z)

            if not all(math.isfinite(v) for v in force):
                raise QsfCoreError('QSF produced non-finite force: %s' % force)
            self.qsf_force_ned = force
            self.qsf_error = ''
            self.controller_source = self.controller_mode
            # force 是 QSF 输出的力，转换回 MAVROS 姿态控制需要的 ENU 净加速度。
            return [
                force[1] / self.mav_mass,
                force[0] / self.mav_mass,
                -force[2] / self.mav_mass - self.gravity,
            ]
        except (QsfCoreError, ZeroDivisionError, ValueError) as exc:
            self.qsf_error = str(exc)
            self.get_logger().warning('QSF fallback to PD: %s' % self.qsf_error)
            return None

    def build_qsf_refs(self, ref):
        # QSF ref_x/ref_y/ref_z 分别对应 north/east/down，而 ROS 位姿是 ENU x/y/z。
        pos_z = ref.pos[2]
        if not self.qsf_reference_is_load:
            pos_z -= self.cable_length
        ref_x = [ref.pos[1], ref.vel[1], ref.acc[1], ref.jerk[1], ref.snap[1]]
        ref_y = [ref.pos[0], ref.vel[0], ref.acc[0], ref.jerk[0], ref.snap[0]]
        ref_z = [-pos_z, -ref.vel[2], -ref.acc[2], -ref.jerk[2], -ref.snap[2]]
        return ref_x, ref_y, ref_z

    def build_sls_state(self):
        # 状态顺序来自 drown/ROS1 版本：负载位置、摆杆方向、负载速度、摆杆角速度。
        load_pos, load_vel, has_load = self.current_load_state()
        pend = [load_pos[0] - self.pose[0], load_pos[1] - self.pose[1],
                load_pos[2] - self.pose[2]]
        pend = normalize3(pend, [0.0, 0.0, -1.0])
        if has_load:
            rel_vel = [load_vel[0] - self.velocity[0],
                       load_vel[1] - self.velocity[1],
                       load_vel[2] - self.velocity[2]]
            pend_rate = cross(pend, rel_vel)
        else:
            pend_rate = [0.0, 0.0, 0.0]

        return [
            load_pos[1], load_pos[0], -load_pos[2],
            pend[1], pend[0], -pend[2],
            load_vel[1], load_vel[0], -load_vel[2],
            pend_rate[1], pend_rate[0], -pend_rate[2],
        ]

    def current_load_state(self):
        # 没有负载位姿时，用机体正下方 cable_length 处的虚拟负载保持 QSF 输入可用。
        now = self.now_seconds()
        has_load = (
            self.use_load_pose and self.load_pose is not None
            and self.load_pose_time is not None
            and now - self.load_pose_time <= self.load_pose_stale_timeout
        )
        if has_load:
            return list(self.load_pose), list(self.load_velocity), True
        return [
            self.pose[0], self.pose[1], self.pose[2] - self.cable_length,
        ], list(self.velocity), False

    def integrate_qsf_xi(self, xi_dot, dt):
        if dt <= 0.0:
            return
        for i in range(3):
            value = self.qsf_xi[i] + xi_dot[i] * dt
            self.qsf_xi[i] = clamp(
                value, -self.qsf_integral_limit, self.qsf_integral_limit)

    def estimate_cable_force_on_drone(self):
        # residual 风估计需要扣掉吊载水平拉力，避免把绳摆误当成风扰。
        if (
            self.pose is None
            or self.load_pose is None
            or self.load_pose_time is None
            or self.load_mass <= 0.0
            or self.now_seconds() - self.load_pose_time > self.load_pose_stale_timeout
        ):
            return [0.0, 0.0, 0.0], False
        cable = [
            self.load_pose[0] - self.pose[0],
            self.load_pose[1] - self.pose[1],
            self.load_pose[2] - self.pose[2],
        ]
        direction = normalize3(cable, [0.0, 0.0, -1.0])
        down_component = max(0.15, -direction[2])
        tension = self.load_mass * self.gravity / down_component
        return [tension * direction[0], tension * direction[1], 0.0], True

    def effective_wind_gain(self, now):
        target_gain = max(0.0, float(self.wind_compensation_gain))
        if target_gain <= 0.0 or self.start_time is None:
            self.effective_wind_compensation_gain = 0.0
            return self.effective_wind_compensation_gain
        elapsed = max(0.0, now - self.start_time)
        if elapsed < self.wind_compensation_warmup_time:
            self.effective_wind_compensation_gain = 0.0
            return self.effective_wind_compensation_gain
        if self.wind_compensation_ramp_time <= 0.0:
            self.effective_wind_compensation_gain = target_gain
            return self.effective_wind_compensation_gain
        ramp = clamp(
            (elapsed - self.wind_compensation_warmup_time)
            / self.wind_compensation_ramp_time,
            0.0,
            1.0,
        )
        self.effective_wind_compensation_gain = target_gain * ramp
        return self.effective_wind_compensation_gain

    def apply_anti_wind(self, ref, accel, dt):
        # 抗风只作用于水平 x/y，z 轴扰动和积分强制清零。
        if dt <= 0.0:
            return accel
        compensation_gain = self.effective_wind_gain(self.now_seconds())
        observed = self.pose
        compensated = list(accel)
        compensation_accel = [0.0, 0.0, 0.0]
        for axis in range(3):
            if axis == 2:
                self.disturbance[axis] = 0.0
                self.wind_integral[axis] = 0.0
                continue
            if (
                self.wind_estimator_mode == 'actual_feedback'
                and self.actual_wind_time is not None
            ):
                self.update_actual_feedback_wind_axis(axis, dt)
            elif self.wind_estimator_mode in ('residual', 'actual_feedback'):
                self.update_residual_wind_axis(axis, dt)
            else:
                command = (
                    self.last_commanded_accel[axis]
                    if self.last_commanded_accel_valid else accel[axis]
                )
                self.disturbance[axis] = self.leso[axis].update(
                    observed[axis], command, dt)
            self.wind_integral[axis] += (ref.pos[axis] - observed[axis]) * dt
            self.wind_integral[axis] = clamp(
                self.wind_integral[axis],
                -self.wind_integral_limit, self.wind_integral_limit)
            residual = (
                self.disturbance[axis]
                - self.wind_integral_gain * self.wind_integral[axis]
            )
            compensation_accel[axis] = -compensation_gain * residual
            compensated[axis] += compensation_accel[axis]

        self.update_wind_compensation_debug(compensation_accel)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        estimate_force = self.wind_estimate_force_n()
        msg.vector.x = estimate_force[0]
        msg.vector.y = estimate_force[1]
        msg.vector.z = estimate_force[2]
        self.wind_estimate_pub.publish(msg)
        self.maybe_log_wind_debug()
        return compensated

    def update_residual_wind_axis(self, axis, dt):
        if not self.observed_accel_valid or not self.last_commanded_accel_valid:
            return
        raw = self.observed_accel[axis] - self.last_commanded_accel[axis]
        cable_force, cable_force_valid = self.estimate_cable_force_on_drone()
        if cable_force_valid and axis < 2:
            raw -= cable_force[axis] / self.mav_mass
        if self.wind_estimate_force_limit > 0.0:
            accel_limit = self.wind_estimate_force_limit / self.mav_mass
            raw = clamp(raw, -accel_limit, accel_limit)
        alpha = 1.0 - math.exp(-dt / self.wind_estimate_filter_tau)
        self.disturbance[axis] += alpha * (raw - self.disturbance[axis])

    def update_actual_feedback_wind_axis(self, axis, dt):
        target_force = self.actual_wind_force[axis]
        if self.wind_estimate_force_limit > 0.0:
            target_force = clamp(
                target_force,
                -self.wind_estimate_force_limit,
                self.wind_estimate_force_limit,
            )
        target_accel = target_force / self.mav_mass
        alpha = 1.0 - math.exp(-dt / self.wind_estimate_filter_tau)
        self.disturbance[axis] += alpha * (target_accel - self.disturbance[axis])

    def wind_estimate_force_n(self):
        return [
            self.disturbance[0] * self.mav_mass,
            self.disturbance[1] * self.mav_mass,
            0.0,
        ]

    def wind_estimate_rel_error_xy(self):
        estimate = self.wind_estimate_force_n()
        err_x = (
            abs(estimate[0] - self.actual_wind_force[0]) / abs(self.actual_wind_force[0])
            if abs(self.actual_wind_force[0]) > 1.0e-6 else 0.0
        )
        err_y = (
            abs(estimate[1] - self.actual_wind_force[1]) / abs(self.actual_wind_force[1])
            if abs(self.actual_wind_force[1]) > 1.0e-6 else 0.0
        )
        return [err_x, err_y, 0.0]

    def update_wind_compensation_debug(self, compensation_accel):
        self.wind_compensation_accel = list(compensation_accel)
        self.wind_compensation_force = [
            self.mav_mass * value for value in self.wind_compensation_accel]
        self.wind_compensation_magnitude = norm3(self.wind_compensation_force)
        self.wind_compensation_direction = unit_vector(self.wind_compensation_force)

    def maybe_log_wind_debug(self):
        if not self.enable_wind_debug_log:
            return
        now = self.now_seconds()
        if now - self.last_wind_debug_log_time < self.wind_debug_log_period:
            return
        self.last_wind_debug_log_time = now

        actual_age = None
        if self.actual_wind_time is not None:
            actual_age = now - self.actual_wind_time
        actual_age_text = 'none' if actual_age is None else '%.2fs' % actual_age
        estimate_force = self.wind_estimate_force_n()
        estimate_error = self.wind_estimate_rel_error_xy()
        self.get_logger().info(
            'wind estimate force_n=(%.3f, %.3f, %.3f) err_xy=(%.2f, %.2f); '
            'wind compensation accel_mps2=(%.3f, %.3f, %.3f) '
            'equiv_force_n=(%.3f, %.3f, %.3f) mag_n=%.3f dir=(%.3f, %.3f, %.3f); '
            'actual_wind_n=(%.3f, %.3f, %.3f) mag_n=%.3f dir=(%.3f, %.3f, %.3f) age=%s'
            % (
                estimate_force[0],
                estimate_force[1],
                estimate_force[2],
                estimate_error[0],
                estimate_error[1],
                self.wind_compensation_accel[0],
                self.wind_compensation_accel[1],
                self.wind_compensation_accel[2],
                self.wind_compensation_force[0],
                self.wind_compensation_force[1],
                self.wind_compensation_force[2],
                self.wind_compensation_magnitude,
                self.wind_compensation_direction[0],
                self.wind_compensation_direction[1],
                self.wind_compensation_direction[2],
                self.actual_wind_force[0],
                self.actual_wind_force[1],
                self.actual_wind_force[2],
                self.actual_wind_magnitude,
                self.actual_wind_direction[0],
                self.actual_wind_direction[1],
                self.actual_wind_direction[2],
                actual_age_text,
            ))

    def limit_accel(self, accel):
        accel[0] = clamp(accel[0], -self.max_acc_xy, self.max_acc_xy)
        accel[1] = clamp(accel[1], -self.max_acc_xy, self.max_acc_xy)
        accel[2] = clamp(accel[2], -self.max_acc_z, self.max_acc_z)

        total = norm3(accel)
        if total > self.max_total_acc:
            scale = self.max_total_acc / total
            accel = [accel[0] * scale, accel[1] * scale, accel[2] * scale]

        horizontal = math.sqrt(accel[0] * accel[0] + accel[1] * accel[1])
        vertical_thrust = max(1.0e-6, self.gravity + accel[2])
        max_horizontal = math.tan(self.max_tilt_rad) * vertical_thrust
        if horizontal > max_horizontal:
            scale = max_horizontal / horizontal
            accel[0] *= scale
            accel[1] *= scale
        return accel

    def build_attitude_target(self, accel):
        quat = attitude_from_accel(accel, self.yaw, self.gravity)
        thrust_vector_norm = norm3([accel[0], accel[1], accel[2] + self.gravity])
        thrust = self.hover_thrust * thrust_vector_norm / self.gravity
        thrust = clamp(thrust, self.min_thrust, self.max_thrust)

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        msg.thrust = float(thrust)
        return msg

    def build_reference_pose(self, ref_pos):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = ref_pos[0]
        msg.pose.position.y = ref_pos[1]
        msg.pose.position.z = ref_pos[2]
        msg.pose.orientation.w = 1.0
        return msg

    def publish_takeoff_position(self, msg):
        if self.flight_stage == 'circle':
            return
        if self.dry_run:
            return
        if not self.enable_takeoff_position_setpoint:
            return
        self.takeoff_pose_pub.publish(msg)

    def maybe_request_offboard_and_arm(self, now):
        if self.dry_run or not self.enable_mavros_services:
            return
        if self.auto_offboard and self.mavros_state.mode != self.offboard_mode:
            if now - self.last_mode_request_time >= self.service_retry_period:
                self.last_mode_request_time = now
                if self.set_mode_client.service_is_ready():
                    req = SetMode.Request()
                    req.custom_mode = self.offboard_mode
                    self.set_mode_client.call_async(req)
        if self.auto_arm and not self.mavros_state.armed:
            if now - self.last_arm_request_time >= self.service_retry_period:
                self.last_arm_request_time = now
                if self.arming_client.service_is_ready():
                    req = CommandBool.Request()
                    req.value = True
                    self.arming_client.call_async(req)

    def real_setpoint_gate(self):
        if self.flight_stage != 'circle':
            return False, self.flight_stage
        if self.dry_run:
            return False, 'dry_run'
        if not self.enable_real_setpoint:
            return False, 'real_setpoint_disabled'
        if self.require_connected and not self.mavros_state.connected:
            return False, 'mavros_not_connected'
        if self.require_offboard and self.mavros_state.mode != self.offboard_mode:
            return False, 'not_offboard'
        if self.require_armed and not self.mavros_state.armed:
            return False, 'not_armed'
        return True, 'real_setpoint_active'

    def publish_status(self, now, reason, ref=None, accel=None, real_active=False):
        if not hasattr(self, '_last_status_time'):
            self._last_status_time = 0.0
        if now - self._last_status_time < self.status_period:
            return
        self._last_status_time = now
        estimate_force = self.wind_estimate_force_n()
        estimate_error = self.wind_estimate_rel_error_xy()
        payload = {
            'reason': reason,
            'flight_stage': self.flight_stage,
            'dry_run': bool(self.dry_run),
            'enable_real_setpoint': bool(self.enable_real_setpoint),
            'real_setpoint_active': bool(real_active),
            'controller_mode': str(self.controller_mode),
            'controller_source': self.controller_source,
            'mav_mass': self.mav_mass,
            'qsf_core_loaded': self.qsf_core is not None,
            'qsf_core_version': self.qsf_core_version,
            'qsf_error': self.qsf_error,
            'qsf_force_ned': self.qsf_force_ned,
            'qsf_xi': self.qsf_xi,
            'pose_received': self.pose is not None,
            'pose_age': None if self.pose_time is None else now - self.pose_time,
            'load_pose_active': self.current_load_state()[2] if self.pose else False,
            'mavros_connected': bool(self.mavros_state.connected),
            'mavros_armed': bool(self.mavros_state.armed),
            'mavros_mode': self.mavros_state.mode,
            'center': self.center,
            'reference': None if ref is None else ref.pos,
            'reference_velocity': None if ref is None else ref.vel,
            'position': self.pose,
            'velocity': self.velocity,
            'accel_cmd': accel,
            'wind_estimator_mode': self.wind_estimator_mode,
            'wind_compensation_gain_target': self.wind_compensation_gain,
            'wind_compensation_gain_effective': self.effective_wind_compensation_gain,
            'wind_compensation_warmup_time': self.wind_compensation_warmup_time,
            'wind_compensation_ramp_time': self.wind_compensation_ramp_time,
            'wind_estimate_force_n': estimate_force,
            'wind_estimate_rel_error_xy': estimate_error,
            'wind_estimate_rel_error_xy_max': max(estimate_error[0], estimate_error[1]),
            'disturbance_estimate': self.disturbance,
            'wind_integral': self.wind_integral,
            'wind_compensation_accel_mps2': self.wind_compensation_accel,
            'wind_compensation_force_n': self.wind_compensation_force,
            'wind_compensation_magnitude_n': self.wind_compensation_magnitude,
            'wind_compensation_direction': self.wind_compensation_direction,
            'actual_wind_force_n': self.actual_wind_force,
            'actual_wind_magnitude_n': self.actual_wind_magnitude,
            'actual_wind_direction': self.actual_wind_direction,
            'actual_wind_age': (
                None if self.actual_wind_time is None
                else now - self.actual_wind_time
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlsCircleController()
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
