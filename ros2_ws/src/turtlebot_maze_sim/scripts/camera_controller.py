#!/usr/bin/env python3
"""Toggle Gazebo Sim 8 overview and robot-follow cameras with C."""
import math
import re
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from ament_index_python.packages import get_package_share_directory


def read_gui_xml(source):
    """Read either a single XML root or Gazebo's multi-root GUI config."""
    text = Path(source).read_text(encoding='utf-8-sig')
    # An XML declaration is not allowed inside the synthetic root.
    text = re.sub(r'^\s*<\?xml\s+[^?]*\?>', '', text, count=1)
    try:
        return ET.fromstring(text), False
    except ET.ParseError as first_error:
        try:
            return ET.fromstring('<gui_fragment>' + text + '</gui_fragment>'), True
        except ET.ParseError as exc:
            raise ValueError(f'Invalid GUI XML in {source}: {exc}') from first_error


class CameraController(Node):
    def __init__(self):
        super().__init__('camera_controller')
        self.declare_parameter('robot_name', 'burger')
        self.declare_parameter('follow_x', -0.45)
        self.declare_parameter('follow_y', 0.0)
        self.declare_parameter('follow_z', 0.35)
        self.declare_parameter('follow_pgain', 1.0)
        self.declare_parameter('overview_config', '')
        self.robot_name = self.get_parameter('robot_name').value
        self.follow_offset = tuple(
            self.get_parameter(f'follow_{axis}').value for axis in ('x', 'y', 'z')
        )
        self.follow_pgain = float(self.get_parameter('follow_pgain').value)
        self.camera_mode = 'overview'
        self.last_key_time = -math.inf
        self.overview_pose = self.load_overview_pose()
        self.keyboard_sub = self.create_subscription(
            Int32, '/keyboard/keypress', self.keyboard_callback, 1
        )
        self.get_logger().info('Camera controller v3 (CameraTrack) ready. Press C in Gazebo to toggle the camera.')

    def load_overview_pose(self):
        override = self.get_parameter('overview_config').value
        config = (Path(override) if override else
                  Path(get_package_share_directory('turtlebot_maze_sim')) / 'config/gazebo/gui.config')
        fallback = (0.0, 0.0, 2.0, 0.0, 1.5708, 0.0)
        try:
            root, _ = read_gui_xml(config)
            for plugin in root.iter('plugin'):
                if plugin.get('filename') == 'MinimalScene':
                    pose = plugin.find('camera_pose')
                    if pose is not None and pose.text:
                        values = tuple(float(v) for v in pose.text.split())
                        if len(values) == 6 and all(math.isfinite(v) for v in values):
                            return values
        except (OSError, ET.ParseError, ValueError) as exc:
            self.get_logger().warning(f'Could not read camera pose: {exc}')
        self.get_logger().warning('Using fallback overview camera pose.')
        return fallback

    def run_gz(self, args, expect_boolean=False):
        try:
            result = subprocess.run(['gz', *args], capture_output=True, text=True, timeout=4.0)
            if result.returncode != 0 or (expect_boolean and 'data: true' not in result.stdout.lower()):
                self.get_logger().warning(f'Gazebo command failed: {result.stdout} {result.stderr}')
                return False
            return True
        except (OSError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f'Gazebo command failed: {exc}')
            return False

    def publish_track(self, request):
        return self.run_gz(['topic', '-t', '/gui/track', '-m', 'gz.msgs.CameraTrack', '-p', request])

    def keyboard_callback(self, msg):
        if msg.data not in (67, 99):
            return
        if time.monotonic() - self.last_key_time < 0.8:
            return
        if self.camera_mode == 'overview':
            if self.enable_follow_camera():
                self.camera_mode = 'follow'
        else:
            if self.enable_overview_camera():
                self.camera_mode = 'overview'
        self.last_key_time = time.monotonic()

    def enable_follow_camera(self):
        x, y, z = self.follow_offset
        request = (
            'track_mode: FOLLOW '
            f'follow_target {{ name: "{self.robot_name}" }} '
            f'follow_offset {{ x: {x} y: {y} z: {z} }} '
            f'follow_pgain: {self.follow_pgain}'
        )
        if not self.publish_track(request):
            return False
        self.get_logger().info('Camera mode: FOLLOW ROBOT')
        return True

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)

    def enable_overview_camera(self):
        # Use the stop-follow service already verified in this installation.
        if not self.run_gz([
            'service', '-s', '/gui/follow', '--reqtype', 'gz.msgs.StringMsg',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', 'data: ""'
        ], expect_boolean=True):
            return False
        x, y, z, roll, pitch, yaw = self.overview_pose
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        request = (f'pose {{ position {{ x: {x} y: {y} z: {z} }} '
                   f'orientation {{ x: {qx} y: {qy} z: {qz} w: {qw} }} }}')
        if not self.run_gz([
            'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
            '--reptype', 'gz.msgs.Boolean', '--timeout', '2000', '--req', request
        ], expect_boolean=True):
            return False
        self.get_logger().info('Camera mode: OVERVIEW')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = CameraController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
