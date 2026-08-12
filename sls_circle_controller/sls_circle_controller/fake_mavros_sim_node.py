"""用于控制器台架测试的 MAVROS 风格点质量仿真节点。"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import AttitudeTarget, State
from rclpy.node import Node

from .controller_math import body_z_from_quat, clamp


class FakeMavrosSim(Node):
    """把控制器的姿态目标积分成假位姿和假 MAVROS 状态。"""

    def __init__(self):
        super().__init__('fake_mavros_sim')
        self._declare_parameters()
        self._read_parameters()

        self.position = [self.initial_x, self.initial_y, self.initial_z]
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation = [0.0, 0.0, 0.0, 1.0]
        self.last_target = None
        self.last_target_time = None
        self.last_update_time = self.now_seconds()

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.state_pub = self.create_publisher(State, self.state_topic, 10)
        self.target_sub = self.create_subscription(
            AttitudeTarget, self.attitude_target_topic,
            self.attitude_target_callback, 10)

        period = 1.0 / max(1.0, self.publish_rate)
        self.timer = self.create_timer(period, self.step)
        self.get_logger().info(
            'fake MAVROS sim started; target_topic=%s'
            % self.attitude_target_topic)

    def _declare_parameters(self):
        defaults = {
            'pose_topic': '/mavros/local_position/pose',
            'state_topic': '/mavros/state',
            'attitude_target_topic': '/sls_circle/debug/attitude_target',
            'frame_id': 'map',
            'publish_rate': 100.0,
            'initial_x': 0.0,
            'initial_y': 0.0,
            'initial_z': 1.0,
            'gravity': 9.80665,
            'hover_thrust': 0.5,
            'linear_drag': 0.18,
            'wind_accel_x': 0.0,
            'wind_accel_y': 0.0,
            'wind_accel_z': 0.0,
            'max_speed': 6.0,
            'target_timeout': 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self):
        for name in [
            'pose_topic', 'state_topic', 'attitude_target_topic', 'frame_id',
            'publish_rate', 'initial_x', 'initial_y', 'initial_z', 'gravity',
            'hover_thrust', 'linear_drag', 'wind_accel_x', 'wind_accel_y',
            'wind_accel_z', 'max_speed', 'target_timeout',
        ]:
            setattr(self, name, self.get_parameter(name).value)
        self.hover_thrust = max(0.05, float(self.hover_thrust))

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def attitude_target_callback(self, msg):
        # 控制器发布的是姿态四元数 + 归一化推力，这里保存为下一步积分输入。
        self.last_target = msg
        self.last_target_time = self.now_seconds()
        self.orientation = [
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        ]

    def step(self):
        now = self.now_seconds()
        dt = max(1.0e-4, now - self.last_update_time)
        self.last_update_time = now

        # 没收到新 setpoint 时只受重力影响，便于暴露控制器断流问题。
        acc = [0.0, 0.0, -self.gravity]
        target_fresh = (
            self.last_target is not None
            and self.last_target_time is not None
            and now - self.last_target_time <= self.target_timeout
        )
        if target_fresh:
            q = self.last_target.orientation
            z_b = body_z_from_quat(q.x, q.y, q.z, q.w)
            # MAVROS AttitudeTarget.thrust 是归一化推力，hover_thrust 用来恢复等效加速度。
            thrust_acc = self.gravity * float(self.last_target.thrust) / self.hover_thrust
            acc = [
                z_b[0] * thrust_acc,
                z_b[1] * thrust_acc,
                z_b[2] * thrust_acc - self.gravity,
            ]

        # 轻量仿真只保留线性阻尼和常值风加速度，避免引入 Gazebo/PX4 内环复杂性。
        acc[0] += self.wind_accel_x - self.linear_drag * self.velocity[0]
        acc[1] += self.wind_accel_y - self.linear_drag * self.velocity[1]
        acc[2] += self.wind_accel_z - self.linear_drag * self.velocity[2]

        for axis in range(3):
            self.velocity[axis] += acc[axis] * dt

        speed = math.sqrt(sum(v * v for v in self.velocity))
        if speed > self.max_speed:
            scale = self.max_speed / speed
            self.velocity = [v * scale for v in self.velocity]

        for axis in range(3):
            self.position[axis] += self.velocity[axis] * dt

        if self.position[2] < 0.05:
            self.position[2] = 0.05
            self.velocity[2] = max(0.0, self.velocity[2])

        self.publish_state()
        self.publish_pose()

    def publish_state(self):
        # 固定发布已连接、已解锁、OFFBOARD，模拟控制器进入闭环所需的最低 MAVROS 状态。
        msg = State()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        msg.armed = True
        msg.guided = True
        msg.manual_input = False
        msg.mode = 'OFFBOARD'
        msg.system_status = 4
        self.state_pub.publish(msg)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.position[0]
        msg.pose.position.y = self.position[1]
        msg.pose.position.z = self.position[2]
        msg.pose.orientation.x = self.orientation[0]
        msg.pose.orientation.y = self.orientation[1]
        msg.pose.orientation.z = self.orientation[2]
        msg.pose.orientation.w = self.orientation[3]
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMavrosSim()
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
