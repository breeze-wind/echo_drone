"""Coordinate helpers for bridging the legacy controller to MAVROS.

The old pymavlink node wrote NED-like payloads directly to PX4 and mixed in a
few sign/height offsets locally.  MAVROS accepts ROS ENU messages and converts
them internally before sending MAVLink.  The helpers in this file intentionally
apply the inverse transform so existing `/robot/*` producers can keep their old
semantics while the transport moves to MAVROS.
"""

import math

LEGACY_COORDINATE_MODE = 'legacy_ned_compatible'
MAVROS_ENU_COORDINATE_MODE = 'mavros_enu'


def ned_xyz_to_mavros_enu(x_ned, y_ned, z_ned):
    """Return the ROS ENU vector that MAVROS converts back to the NED input."""
    return y_ned, x_ned, -z_ned


def legacy_position_to_mavros_enu(x, y, z):
    """Convert the old x, -y, -z MAVLink payload convention to MAVROS ENU."""
    return ned_xyz_to_mavros_enu(x, -y, -z)


def legacy_vision_pose_position(x, y, z, z_offset):
    """Convert `/robot/current_pose` position into MAVROS vision-pose ENU."""
    return legacy_position_to_mavros_enu(x, y, z + z_offset)


def legacy_target_position(x, y, z, reference_z_offset):
    """Convert `/robot/target_pose` position into MAVROS local setpoint ENU."""
    return legacy_position_to_mavros_enu(x, y, z - reference_z_offset)


def legacy_nav_velocity(cmd_x, cmd_y, current_height, target_height, pid_height):
    """Preserve the old navigation velocity convention used by `/cmd_vel`."""
    vx_ned = cmd_x
    vy_ned = -cmd_y
    vz_ned = -pid_height * (target_height - current_height)
    return ned_xyz_to_mavros_enu(vx_ned, vy_ned, vz_ned)


def legacy_passing_door_velocity(cmd_x, cmd_y, current_height, target_height, pid_height):
    """Preserve the old special velocity axes used while passing the door."""
    vx_ned = cmd_y
    vy_ned = cmd_x
    vz_ned = -pid_height * (target_height - current_height)
    return ned_xyz_to_mavros_enu(vx_ned, vy_ned, vz_ned)


def mavros_enu_yaw_for_legacy_ned_yaw(yaw_ned):
    """Return ENU yaw that MAVROS converts back to the legacy NED yaw."""
    return math.pi / 2.0 - yaw_ned


def quaternion_from_euler_xyzw(roll, pitch, yaw):
    """Build a geometry-msg-style quaternion tuple from roll, pitch, yaw."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def euler_from_quaternion_msg(quaternion_msg):
    """Extract roll, pitch, yaw from a geometry_msgs Quaternion-like object."""
    x = quaternion_msg.x
    y = quaternion_msg.y
    z = quaternion_msg.z
    w = quaternion_msg.w

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def legacy_vision_orientation_to_mavros_enu(quaternion_msg):
    """Convert old vision orientation into the ENU orientation MAVROS expects."""
    roll, pitch, yaw = euler_from_quaternion_msg(quaternion_msg)
    return quaternion_from_euler_xyzw(roll, pitch, math.pi / 2.0 + yaw)
