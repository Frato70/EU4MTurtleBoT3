#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class GroundTruthOdometry(Node):

    def __init__(self):
        super().__init__('ground_truth_odometry')

        self.declare_parameter(
            'input_topic', '/ground_truth_pose'
        )
        self.declare_parameter(
            'output_topic', '/odom_ground_truth'
        )

        input_topic = self.get_parameter(
            'input_topic'
        ).value

        output_topic = self.get_parameter(
            'output_topic'
        ).value

        self.odom_pub = self.create_publisher(
            Odometry,
            output_topic,
            10
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            input_topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f'Ground truth visualization: '
            f'{input_topic} -> {output_topic}'
        )

    def pose_callback(self, msg):

        odom = Odometry()

        # Preserve original timestamp and reference frame
        odom.header = msg.header
        odom.header.frame_id = 'odom'

        # This is a visualization message, not wheel odometry
        odom.child_frame_id = 'base_footprint'

        # Exact pose received from Gazebo
        odom.pose.pose = msg.pose

        # Twist and covariances are left at zero.
        # They are not measured by this converter.

        self.odom_pub.publish(odom)


def main(args=None):

    rclpy.init(args=args)

    node = GroundTruthOdometry()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
