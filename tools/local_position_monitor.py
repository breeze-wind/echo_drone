#!/usr/bin/env python3
"""持续监测 MAVROS 本地位置的频率、新鲜度和数值有效性。"""

import argparse
import math
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/mavros/mavros/pose')
    parser.add_argument('--min-rate', type=float, default=5.0)
    parser.add_argument('--stale-timeout', type=float, default=1.0)
    parser.add_argument('--report-period', type=float, default=1.0)
    parser.add_argument('--window', type=float, default=5.0)
    return parser.parse_args()


class LocalPositionMonitor:
    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.arrivals = deque()
        self.last_receive = None
        self.last_pose = None
        self.invalid_reason = '尚未收到消息'
        self.ever_failed = False
        self.subscription = node.create_subscription(
            PoseStamped, args.topic, self._on_pose, 20)
        self.timer = node.create_timer(args.report_period, self._report)

    def _on_pose(self, msg):
        now = time.monotonic()
        self.last_receive = now
        self.last_pose = msg
        self.arrivals.append(now)
        self._trim(now)

        values = (
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            self.invalid_reason = '位置或姿态包含 NaN/Inf'
            return
        q = msg.pose.orientation
        q_norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not 0.5 <= q_norm <= 1.5:
            self.invalid_reason = '四元数模长异常 %.3f' % q_norm
            return
        self.invalid_reason = ''

    def _trim(self, now):
        cutoff = now - self.args.window
        while self.arrivals and self.arrivals[0] < cutoff:
            self.arrivals.popleft()

    def _rate(self, now):
        self._trim(now)
        if len(self.arrivals) < 2:
            return 0.0
        span = self.arrivals[-1] - self.arrivals[0]
        return (len(self.arrivals) - 1) / span if span > 0.0 else 0.0

    def _report(self):
        now = time.monotonic()
        rate = self._rate(now)
        age = math.inf if self.last_receive is None else now - self.last_receive
        reasons = []
        if age > self.args.stale_timeout:
            reasons.append('消息超时')
        if rate < self.args.min_rate:
            reasons.append('频率过低')
        if self.invalid_reason:
            reasons.append(self.invalid_reason)

        status = 'OK' if not reasons else 'FAIL'
        self.ever_failed = self.ever_failed or bool(reasons)
        age_text = 'never' if math.isinf(age) else '%.3fs' % age
        pose_text = ''
        if self.last_pose is not None:
            p = self.last_pose.pose.position
            pose_text = ' xyz=(%.3f, %.3f, %.3f)' % (p.x, p.y, p.z)
        detail = '' if not reasons else ' reason=' + ','.join(reasons)
        print('[%s] topic=%s rate=%.1fHz age=%s%s%s' % (
            status, self.args.topic, rate, age_text, pose_text, detail), flush=True)


def main():
    args = parse_args()
    if args.min_rate <= 0 or args.stale_timeout <= 0 or args.report_period <= 0:
        print('阈值和周期必须大于 0', file=sys.stderr)
        return 2
    rclpy.init()
    node = rclpy.create_node('local_position_monitor')
    monitor = LocalPositionMonitor(node, args)
    print('监测 %s：最低 %.1fHz，超时 %.2fs；Ctrl-C 退出。' % (
        args.topic, args.min_rate, args.stale_timeout), flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 1 if monitor.ever_failed else 0


if __name__ == '__main__':
    sys.exit(main())
