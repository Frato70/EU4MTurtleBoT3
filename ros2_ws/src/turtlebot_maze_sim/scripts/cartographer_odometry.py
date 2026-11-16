#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class CartographerOdometry(Node):

    def __init__(self):
        super().__init__('cartographer_odometry')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.pose_callback,
            10
        )

        self.publisher = self.create_publisher(
            Odometry,
            '/odom_cartographer',
            10
        )

        self.get_logger().info(
            'Cartographer visualization: '
            '/tracked_pose -> /odom_cartographer'
        )

    def pose_callback(self, msg):
        odom = Odometry()

        # Preserve Cartographer's reference frame and timestamp
        odom.header = msg.header

        # /tracked_pose represents the tracking_frame (imu_link)
        odom.child_frame_id = 'imu_link'

        odom.pose.pose = msg.pose

        # Velocities are not available from PoseStamped.
        # Twist remains zero; visualization only.

        self.publisher.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CartographerOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
