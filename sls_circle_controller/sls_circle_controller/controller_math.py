"""MAVROS 姿态 setpoint 控制器共用的小型数学工具。"""

import math


def clamp(value, low, high):
    return max(low, min(high, value))


def norm3(v):
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def normalize3(v, fallback):
    n = norm3(v)
    if n < 1.0e-9:
        return list(fallback)
    return [v[0] / n, v[1] / n, v[2] / n]


def unit_vector(v):
    return normalize3(v, [0.0, 0.0, 0.0])


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def quat_from_rotation_matrix(r):
    """把 3x3 旋转矩阵转成 ROS/MAVROS 使用的 xyzw 四元数。"""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2][1] - r[1][2]) / s
        qy = (r[0][2] - r[2][0]) / s
        qz = (r[1][0] - r[0][1]) / s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        qw = (r[2][1] - r[1][2]) / s
        qx = 0.25 * s
        qy = (r[0][1] + r[1][0]) / s
        qz = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        qw = (r[0][2] - r[2][0]) / s
        qx = (r[0][1] + r[1][0]) / s
        qy = 0.25 * s
        qz = (r[1][2] + r[2][1]) / s
    else:
        s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
        qw = (r[1][0] - r[0][1]) / s
        qx = (r[0][2] + r[2][0]) / s
        qy = (r[1][2] + r[2][1]) / s
        qz = 0.25 * s
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return [qx / n, qy / n, qz / n, qw / n]


def attitude_from_accel(accel_enu, yaw, gravity):
    """根据 ENU 期望加速度生成机体姿态四元数。

    这里的输入是“净加速度命令”，函数内部加上重力后得到总推力方向。
    """
    thrust_vector = [accel_enu[0], accel_enu[1], accel_enu[2] + gravity]
    z_b = normalize3(thrust_vector, [0.0, 0.0, 1.0])
    x_c = [math.cos(yaw), math.sin(yaw), 0.0]
    y_b = normalize3(cross(z_b, x_c), [0.0, 1.0, 0.0])
    x_b = normalize3(cross(y_b, z_b), [1.0, 0.0, 0.0])
    rot = [
        [x_b[0], y_b[0], z_b[0]],
        [x_b[1], y_b[1], z_b[1]],
        [x_b[2], y_b[2], z_b[2]],
    ]
    return quat_from_rotation_matrix(rot)


def body_z_from_quat(qx, qy, qz, qw):
    """从姿态四元数取机体 z 轴在世界系中的方向。"""
    return [
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    ]


class LesoAxis:
    """单轴三阶位置 LESO，用于估计未建模扰动加速度。"""

    def __init__(self, bandwidth):
        self.z1 = 0.0
        self.z2 = 0.0
        self.z3 = 0.0
        self.set_bandwidth(bandwidth)
        self.initialized = False

    def set_bandwidth(self, bandwidth):
        self.bandwidth = max(0.1, float(bandwidth))
        self.beta1 = 3.0 * self.bandwidth
        self.beta2 = 3.0 * self.bandwidth * self.bandwidth
        self.beta3 = self.bandwidth * self.bandwidth * self.bandwidth

    def reset(self, position, velocity=0.0):
        self.z1 = float(position)
        self.z2 = float(velocity)
        self.z3 = 0.0
        self.initialized = True

    def update(self, position, nominal_accel, dt):
        """用当前位置、名义加速度和 dt 更新扰动估计 z3。"""
        if not self.initialized:
            self.reset(position)
        if dt <= 0.0:
            return self.z3
        e = float(position) - self.z1
        self.z1 += (self.z2 + self.beta1 * e) * dt
        self.z2 += (self.z3 + self.beta2 * e + float(nominal_accel)) * dt
        self.z3 += self.beta3 * e * dt
        return self.z3
