#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry


class OdomCovariance(Node):

    def __init__(self):
        super().__init__('odom_covariance')

        self.publisher = self.create_publisher(
            Odometry, '/odom_cov', 10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom_raw',
            self.odom_callback,
            qos_profile_sensor_data
        )

    def odom_callback(self, msg):

        # Twist covariance order:
        # vx, vy, vz, wx, wy, wz

        covariance = [0.0] * 36

        covariance[0] = 0.0025   # vx
        covariance[7] = 0.0004   # vy
        covariance[14] = 1.0     # vz
        covariance[21] = 1.0     # wx
        covariance[28] = 1.0     # wy
        covariance[35] = 0.01    # wz

        msg.twist.covariance = covariance

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = OdomCovariance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
