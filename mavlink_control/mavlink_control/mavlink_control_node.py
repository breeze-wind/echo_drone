from pymavlink import mavutil
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from std_msgs.msg import Bool

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class MavlinkControl(Node):

    def __init__(self):
        super().__init__('mavlink_control_node')

        self.declare_parameter('cruise_height', 0.6)

        self.cruise_height = self.get_parameter('cruise_height').get_parameter_value().double_value

        self.target_frame = "map" #世界系
        self.source_frame = "livox" #机体系

        self.pid_height = 0.4
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.ready_to_arm = False #是否准备解锁
        self.arming_state = False #飞控解锁状态
        self.land_state = False #降落状态

        self.if_is_flying = False

        # Connect to PX4 over serial or UDP
        self.master = mavutil.mavlink_connection('/dev/ttyACM0')
        self.master.wait_heartbeat()
        print("Connected")

        #请求飞控以5Hz发送RC_CHANNELS信息
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            5,  # Hz
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
        self.landing_state_sub = self.create_subscription(
            Bool,
            '/robot/landing_state',
            self.landing_state_callback,
            10) #
        self.target_pose_sub  # prevent unused variable warning
        self.cmd_vel_sub
        self.landing_state_sub

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #发送自身当前测量位置, 10Hz
        self.vision_pose_timer = self.create_timer(0.1, self.vision_pose_timer_callback)
        #读取遥控杆位置
        self.channel_position_timer = self.create_timer(0.2, self.channel_position_timer_callback)
        #控制起飞降落动作
        self.mode_control_timer = self.create_timer(1.0, self.mode_control_timer_callback)
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
        self.current_x, self.current_y, self.current_z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        # self.current_x, self.current_y, self.current_z = 0.0, 0.0, 1.0
        roll, pitch, yaw = 0.0, 0.0, 0.0  # Orientation in radians

        self.master.mav.vision_position_estimate_send(
            timestamp_us,
            self.current_x, self.current_y, self.current_z,
            roll, pitch, yaw
        )

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

        # self.get_logger().info('mavlink: send vision estimate pose x y z: %f, %f, %f' %(self.current_x, self.current_y, self.current_z))

    #读取遥控杆位置
    def channel_position_timer_callback(self):
        if not self.ready_to_arm:
            channels = self.master.recv_match(type=['RC_CHANNELS', 'RC_CHANNELS_RAW'], blocking=False)
            if channels:
                # RC_CHANNELS gives chan1_raw to chan8_raw (and up to chan18_raw)
                if channels.get_type() == 'RC_CHANNELS':
                    print(f"Chan1: {channels.chan1_raw}")
                    print(f"Chan2: {channels.chan2_raw}")
                    print(f"Chan3: {channels.chan3_raw}")
                    print(f"Chan4: {channels.chan4_raw}")
                    print(' ')
                    if channels.chan1_raw < 1100:
                        if channels.chan2_raw > 1930:
                            if channels.chan3_raw < 1100:
                                if channels.chan4_raw > 1930:
                                    self.ready_to_arm = True #可以起飞
                                    return
                    self.ready_to_arm = False

    #监测心跳信号，是否已经解锁
    def state_timer_callback(self):
        hb = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if hb:
            armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            print(f"Vehicle armed? {armed}")
            print('')
            msg = Bool() #给决策发送是否解锁
            if armed:
                self.arming_state = True
            else:
                self.arming_state = False
            msg.data = self.arming_state
            self.arm_state_pub.publish(msg)

        #如果可以起飞就发送解锁命令
        if not self.arming_state:
            if self.ready_to_arm:
                if not self.if_is_flying:
                    self.master.mav.command_long_send(
                        self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        0,
                        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.6
                    )
                    if_is_flying = True

    #控制起飞降落动作
    def mode_control_timer_callback(self):
        if self.land_state:
            if self.if_is_flying:
                self.get_logger().info('进入降落状态')
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,  # Confirmation
                    0, 0, 0, 0, 0, 0, 0.2  # 参数（俯仰角、纬度、经度、高度等）
                )
                self.if_is_flying = False
        # elif self.arming_state:
        #     if self.ready_to_arm: #可以起飞
        #         if not self.if_is_flying:
        #             self.get_logger().info('-------------起飞--------------')
        #             self.master.mav.command_long_send(
        #                 self.master.target_system, self.master.target_component,
        #                 mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        #                 0,  # Confirmation
        #                 0, 0, 0, 0, 0, 0, -self.cruise_height  # 参数（俯仰角、纬度、经度、高度等）
        #             )
                    # self.if_is_flying = True

    #发送teb速度到飞控
    def cmd_vel_callback(self, msg):
        # Send vision speed estimate
        timestamp_us = int(time.time() * 1e6)
        vx, vy, vz = msg.data.x, msg.data.y, self.pid_height * (self.cruise_height - self.current_z)   # Speed in m/s

        self.master.mav.set_position_target_local_ned(
            timestamp_us,
            self.master.target_system, self.master.target_component,
            vx, vy, vz
        )

        self.get_logger().info('mavlink: send vision speed vx vy vz: %f, %f, %f' %(vx, vy, vz))

    #发送目标点位置到飞控
    def target_pose_callback(self, msg):
        #发送定点指令给飞控
        time_boot_ms = int(time.time()*1000) & 0xFFFFFFFF
        type_mask = (
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        x, y ,z = 0.0, 0.0, 1.0
        # x, y ,z = msg.data.x, msg.data.y, msg.data.z
        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000000000000000,
            x, y ,z,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        self.get_logger().info('mavlink: send target pose x y z: %f, %f, %f' %(x, y, z))

    def landing_state_callback(self, msg):
        self.land_state = msg.data

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
