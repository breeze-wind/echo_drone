"""Split MAVROS topic-name collisions into uniquely typed bag topics."""

import rclpy
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn, RCOut
from rclpy.node import Node


class MavrosAmbiguousTopicSplitter(Node):
    """Republish both types carried by MAVROS' colliding in/out names."""

    def __init__(self):
        super().__init__('mavros_ambiguous_topic_splitter')

        self.rc_in_pub = self.create_publisher(
            RCIn, '/bag_split/mavros/rc_in', 10)
        self.odom_in_pub = self.create_publisher(
            Odometry, '/bag_split/mavros/odometry_in', 10)
        self.rc_out_pub = self.create_publisher(
            RCOut, '/bag_split/mavros/rc_out', 10)
        self.odom_out_pub = self.create_publisher(
            Odometry, '/bag_split/mavros/odometry_out', 10)

        self.create_subscription(
            RCIn, '/mavros/mavros/in', self.rc_in_pub.publish, 10)
        self.create_subscription(
            Odometry, '/mavros/mavros/in', self.odom_in_pub.publish, 10)
        self.create_subscription(
            RCOut, '/mavros/mavros/out', self.rc_out_pub.publish, 10)
        self.create_subscription(
            Odometry, '/mavros/mavros/out', self.odom_out_pub.publish, 10)

        self.get_logger().info(
            'splitting MAVROS in/out collisions into /bag_split/mavros/*')


def main(args=None):
    rclpy.init(args=args)
    node = MavrosAmbiguousTopicSplitter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
