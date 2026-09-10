"""Measure SLS Gazebo tracking stability and return a CI-friendly result."""

import argparse
import json
import math
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RunningMetric:
    def __init__(self):
        self.count = 0
        self.sum_sq = 0.0
        self.maximum = 0.0

    def add(self, value):
        value = abs(float(value))
        self.count += 1
        self.sum_sq += value * value
        self.maximum = max(self.maximum, value)

    @property
    def rms(self):
        return math.sqrt(self.sum_sq / self.count) if self.count else math.inf


class GazeboStabilityValidator(Node):
    """Validate tracking from the controller's JSON status stream."""

    def __init__(self, args):
        super().__init__('gazebo_stability_validator')
        self.args = args
        self.started_at = self.now_seconds()
        self.last_status_at = None
        self.status_count = 0
        self.invalid_count = 0
        self.first_invalid_status = None
        self.first_invalid_reason = None
        self.xy_error = RunningMetric()
        self.z_error = RunningMetric()
        self.speed = RunningMetric()
        self.wind_error = RunningMetric()
        self.min_control_hz = math.inf
        self.min_pose_hz = math.inf
        self.max_pose_age = 0.0
        self.qsf_errors = set()
        self.last_status = None
        self.result = None

        self.create_subscription(String, args.status_topic, self.status_callback, 20)
        self.create_timer(0.1, self.tick)

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def status_callback(self, msg):
        now = self.now_seconds()
        self.last_status_at = now
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            self.invalid_count += 1
            if self.first_invalid_status is None:
                self.first_invalid_status = msg.data
                self.first_invalid_reason = str(exc)
            return
        self.last_status = status
        self.status_count += 1
        if now - self.started_at < self.args.warmup:
            return

        position = status.get('position')
        reference = status.get('reference')
        velocity = status.get('velocity')
        if not self.finite_vector(position) or not self.finite_vector(reference):
            self.invalid_count += 1
            return

        px, py, pz = self.vector_components(position)
        rx, ry, rz = self.vector_components(reference)
        dx = px - rx
        dy = py - ry
        dz = pz - rz
        self.xy_error.add(math.hypot(dx, dy))
        self.z_error.add(dz)
        if self.finite_vector(velocity):
            vx, vy, vz = self.vector_components(velocity)
            self.speed.add(math.sqrt(vx ** 2 + vy ** 2 + vz ** 2))

        self.min_control_hz = min(
            self.min_control_hz, float(status.get('actual_control_hz', 0.0)))
        self.min_pose_hz = min(
            self.min_pose_hz, float(status.get('pose_input_hz', 0.0)))
        self.max_pose_age = max(
            self.max_pose_age, float(status.get('pose_age') or math.inf))
        qsf_error = status.get('qsf_error')
        if qsf_error:
            self.qsf_errors.add(str(qsf_error))
        wind_error = status.get('wind_estimate_rel_error_xy_max')
        if isinstance(wind_error, (int, float)) and math.isfinite(wind_error):
            self.wind_error.add(wind_error)

    @staticmethod
    def finite_vector(value):
        if isinstance(value, dict):
            components = [value.get(axis) for axis in ('x', 'y', 'z')]
        elif isinstance(value, (list, tuple)) and len(value) >= 3:
            components = value[:3]
        else:
            return False
        return all(
            isinstance(component, (int, float)) and math.isfinite(component)
            for component in components)

    @staticmethod
    def vector_components(value):
        if isinstance(value, dict):
            return value['x'], value['y'], value['z']
        return value[0], value[1], value[2]

    def tick(self):
        elapsed = self.now_seconds() - self.started_at
        if self.last_status_at is not None and self.now_seconds() - self.last_status_at > 1.0:
            self.finish('status stream became stale')
        elif elapsed >= self.args.duration:
            self.finish(None)

    def finish(self, fatal_reason):
        if self.result is not None:
            return
        reasons = []
        measured_seconds = max(0.0, self.args.duration - self.args.warmup)
        minimum_samples = max(5, int(measured_seconds * 2.0))
        if fatal_reason:
            reasons.append(fatal_reason)
        if self.xy_error.count < minimum_samples:
            reasons.append('not enough valid status samples')
        if self.invalid_count:
            reasons.append('invalid or non-finite status samples: %d' % self.invalid_count)
        checks = [
            ('xy_error_rms_m', self.xy_error.rms, self.args.max_xy_rms),
            ('xy_error_max_m', self.xy_error.maximum, self.args.max_xy_max),
            ('z_error_rms_m', self.z_error.rms, self.args.max_z_rms),
            ('z_error_max_m', self.z_error.maximum, self.args.max_z_max),
            ('speed_max_mps', self.speed.maximum, self.args.max_speed),
        ]
        for name, value, limit in checks:
            if not math.isfinite(value) or value > limit:
                reasons.append('%s %.4f exceeds %.4f' % (name, value, limit))
        if self.min_control_hz < self.args.min_control_hz:
            reasons.append('control_hz %.2f below %.2f' % (
                self.min_control_hz, self.args.min_control_hz))
        if self.min_pose_hz < self.args.min_pose_hz:
            reasons.append('pose_hz %.2f below %.2f' % (
                self.min_pose_hz, self.args.min_pose_hz))
        if self.max_pose_age > self.args.max_pose_age:
            reasons.append('pose_age %.4f exceeds %.4f' % (
                self.max_pose_age, self.args.max_pose_age))
        if self.args.require_qsf and self.qsf_errors:
            reasons.append('QSF errors: %s' % ', '.join(sorted(self.qsf_errors)))

        report = {
            'pass': not reasons,
            'duration_s': self.args.duration,
            'warmup_s': self.args.warmup,
            'samples': self.xy_error.count,
            'xy_error_rms_m': self.xy_error.rms,
            'xy_error_max_m': self.xy_error.maximum,
            'z_error_rms_m': self.z_error.rms,
            'z_error_max_m': self.z_error.maximum,
            'speed_max_mps': self.speed.maximum,
            'min_control_hz': self.min_control_hz,
            'min_pose_hz': self.min_pose_hz,
            'max_pose_age_s': self.max_pose_age,
            'wind_rel_error_rms': self.wind_error.rms,
            'qsf_errors': sorted(self.qsf_errors),
            'last_position': self.last_status.get('position') if self.last_status else None,
            'last_reference': self.last_status.get('reference') if self.last_status else None,
            'last_velocity': self.last_status.get('velocity') if self.last_status else None,
            'last_qsf_state': self.last_status.get(
                'qsf_state_ned_load_semantics') if self.last_status else None,
            'load_pose_active': bool(
                self.last_status.get('load_pose_active')) if self.last_status else False,
            'first_invalid_reason': self.first_invalid_reason,
            'first_invalid_status': self.first_invalid_status,
            'reasons': reasons,
        }
        print('SLS_GAZEBO_VALIDATION=' + json.dumps(report, sort_keys=True), flush=True)
        self.result = 0 if not reasons else 1


def parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--status-topic', default='/sls_circle/status')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--warmup', type=float, default=8.0)
    parser.add_argument('--max-xy-rms', type=float, default=0.25)
    parser.add_argument('--max-xy-max', type=float, default=0.60)
    parser.add_argument('--max-z-rms', type=float, default=0.15)
    parser.add_argument('--max-z-max', type=float, default=0.35)
    parser.add_argument('--max-speed', type=float, default=2.0)
    parser.add_argument('--min-control-hz', type=float, default=85.0)
    parser.add_argument('--min-pose-hz', type=float, default=120.0)
    parser.add_argument('--max-pose-age', type=float, default=0.10)
    parser.add_argument('--require-qsf', action='store_true')
    return parser.parse_args(argv)


def main(args=None):
    ros_args = rclpy.utilities.remove_ros_args(args=sys.argv if args is None else args)
    parsed = parse_args(ros_args[1:])
    if parsed.warmup >= parsed.duration:
        raise SystemExit('--warmup must be smaller than --duration')
    rclpy.init(args=args)
    node = GazeboStabilityValidator(parsed)
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        result = 2 if node.result is None else node.result
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
