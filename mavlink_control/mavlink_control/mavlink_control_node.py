from pymavlink import mavutil
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from std_msgs.msg import Bool

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# from tf2.transformations import quaternion_from_euler, euler_from_quaternion
import transforms3d as tfs

import serial
import serial.tools.list_ports

class MavlinkControl(Node):

    def __init__(self):
        super().__init__('mavlink_control_node')

        self.declare_parameter('cruise_height', 0.6)

        self.cruise_height = self.get_parameter('cruise_height').get_parameter_value().double_value

        self.target_frame = "map" #世界系
        self.source_frame = "livox" #雷达系

        self.pid_height = 0.65
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.ready_to_arm = False #是否准备解锁
        self.arming_state = False #飞控解锁状态
        self.if_nav = False
        self.current_passing_door = False

        # Connect to PX4 over serial or UDP
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if port.manufacturer == 'CUAV':
                self.master = mavutil.mavlink_connection(port.device, band=230400)
        self.master.wait_heartbeat()
        self.get_logger().info('Connected')

        #请求飞控以10Hz发送RC_CHANNELS信息
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            10,  # Hz
            1   # start (1 to start, 0 to stop)
        )

        self.current_pose_pub = self.create_publisher(PoseStamped, '/robot/current_pose', 10)
        self.arm_state_pub = self.create_publisher(Bool, '/robot/arm_state', 10)
        self.target_pose_sub = self.create_subscription(
            TransformStamped,
            '/robot/target_pose',
            self.target_pose_callback,
            10) #导航定点
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10) #导航规划速度
        self.nav_state_sub = self.create_subscription(
            Bool,
            '/robot/nav_state',
            self.nav_state_callback,
            10) #
        self.passing_door_state_sub = self.create_subscription(
            Bool,
            '/robot/passing_door_state',
            self.passing_door_state_callback,
            10) #
        self.target_pose_sub  # prevent unused variable warning
        self.cmd_vel_sub
        self.nav_state_sub
        self.passing_door_state_sub

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #发送自身当前测量位置, 10Hz
        self.vision_pose_timer = self.create_timer(0.08, self.vision_pose_timer_callback)
        #读取遥控杆位置
        self.channel_position_timer = self.create_timer(0.1, self.channel_position_timer_callback)
        #监测心跳信号，是否已经解锁
        self.state_timer = self.create_timer(1.0, self.state_timer_callback)

    #发送自身当前测量位置到飞控和决策, 10Hz
    def vision_pose_timer_callback(self):
        #获取坐标系变换
        try:
            t = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.source_frame} to {self.target_frame}: {ex}')
            return

        # Send vision pose estimate
        timestamp_us = int(time.time() * 1e6)
        # Position in meters (NED)
        self.current_x, self.current_y, self.current_z = (t.transform.translation.x, t.transform.translation.y,
                                                          t.transform.translation.z)
        roll, pitch, yaw = tfs.euler.quat2euler([t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z], "sxyz")  # Orientation in radians

        self.master.mav.vision_position_estimate_send(
            timestamp_us,
            self.current_x, -self.current_y, -(self.current_z-0.08),
            roll, -pitch, -yaw
        )
        self.get_logger().info('mavlink: send vision estimate pose x y z: %f, %f, %f'
                               % (self.current_x, -self.current_y, -(self.current_z - 0.08)))
        # self.get_logger().info('mavlink: send vision estimate rpy: %f, %f, %f' %(roll, -pitch, -yaw))

        #发送当前位置到决策
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = self.current_x
        msg.pose.position.y = self.current_y
        msg.pose.position.z = self.current_z
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0
        self.current_pose_pub.publish(msg)

    #读取遥控杆位置
    def channel_position_timer_callback(self):
        if not self.ready_to_arm:
            channels = self.master.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'], blocking=False)
            if channels:
                # RC_CHANNELS gives chan1_raw to chan8_raw (and up to chan18_raw)
                if channels.get_type() == 'RC_CHANNELS':
                    self.get_logger().info('Chan1: %d' %channels.chan1_raw)
                    # self.get_logger().info('Chan2: %d' %channels.chan2_raw)
                    # self.get_logger().info('Chan3: %d' %channels.chan3_raw)
                    # self.get_logger().info('Chan4: %d' %channels.chan4_raw)
                    if channels.chan1_raw < 1100:
                        if channels.chan2_raw > 1930:
                            if channels.chan3_raw < 1100:
                                if channels.chan4_raw > 1930:
                                    self.ready_to_arm = True
                                    self.master.mav.command_long_send( #板外解锁命令
                                        self.master.target_system, self.master.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0,
                                        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
                                    )
                                    self.get_logger().info('mavlink: vehicle armed!!!!')
                                    return
                    self.ready_to_arm = False

    #监测心跳信号，是否已经解锁
    def state_timer_callback(self):
        hb = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            self.get_logger().info('????????Vehicle armed?  %d' %armed)
            msg = Bool() #给决策发送是否解锁
            if armed: #飞控实际状态解锁
                self.arming_state = True
            else:
                self.arming_state = False
                if self.ready_to_arm:
                    self.ready_to_arm = False #重新发解锁命令

            msg.data = self.arming_state
            self.arm_state_pub.publish(msg)

    #发送teb速度到飞控
    def cmd_vel_callback(self, msg):
        # Send vision speed estimate
        if self.if_nav:
            time_boot_ms = int(self.get_clock().now().nanoseconds / 1e6) & 0xFFFFFFFF
            if not self.current_passing_door:
                vx, vy, vz = msg.linear.x, -msg.linear.y, -self.pid_height * (self.cruise_height - (self.current_z - 0.08))   # Speed in m/s

                self.master.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    self.master.target_system, self.master.target_component,
                    1,
                    0b0000000111000111,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    0, 0
                )
            else: #穿门状态
                vx, vy, vz = -msg.linear.y, -msg.linear.x, -self.pid_height * (self.cruise_height - self.current_z)   # Speed in m/s

                self.master.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    self.master.target_system, self.master.target_component,
                    1,
                    0b0000010111000111,
                    0, 0, 0,
                    vx, vy, vz,
                    0, 0, 0,
                    -1.57, 0
                )

            self.get_logger().info('mavlink: send vision speed vx vy vz: %f, %f, %f' %(vx, vy, vz))

    #发送目标点位置到飞控
    def target_pose_callback(self, msg):
        #发送定点指令给飞控
        if not self.if_nav:
            time_boot_ms = int(self.get_clock().now().nanoseconds / 1e6) & 0xFFFFFFFF
            x, y ,z = (msg.transform.translation.x, -msg.transform.translation.y,
                       -(msg.transform.translation.z-0.08))
            if not self.current_passing_door:
                self.master.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    self.master.target_system, self.master.target_component,
                    1,
                    0b0000000111111000,
                    x, y, z,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0
                )
            else: #穿门状态
                self.master.mav.set_position_target_local_ned_send(
                    time_boot_ms,
                    self.master.target_system, self.master.target_component,
                    1,
                    0b0000010111111000,
                    x, y, z,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    -1.57, 0.0
                )
            self.get_logger().info('mavlink: send target pose x y z: %f, %f, %f' %(x, y, z))

    def nav_state_callback(self, msg):
        self.if_nav = msg.data

    def passing_door_state_callback(self, msg):
        self.current_passing_door = msg.data

def main(args=None):
    rclpy.init(args=args)

    mavlink_control = MavlinkControl()

    rclpy.spin(mavlink_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mavlink_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
