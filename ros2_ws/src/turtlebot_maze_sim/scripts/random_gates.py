#!/usr/bin/env python3
"""Four kinematic maze gates via Gazebo Sim UserCommands set_pose_vector.

No prismatic joints, PID, Gazebo bridge or force-driven gate movement.
This prototype uses one `gz service` subprocess for ALL FOUR poses at a moderate
frequency (default 20 Hz), rather than four subprocesses per frame.
"""
import math
import random
import subprocess
from dataclasses import dataclass

import rclpy
from rclpy.node import Node


@dataclass
class Gate:
    name: str
    x0: float
    y0: float
    z0: float
    yaw: float
    distance: float
    goal: float
    state: str
    remaining: float
    start_distance: float = 0.0
    motion_elapsed: float = 0.0
    motion_duration: float = 1.0
    max_extension: float = 0.65


class KinematicRandomGates(Node):
    def __init__(self):
        super().__init__('random_gates')
        defaults = {
            'world_name': 'maze_world', 'speed_mps': 0.08,
            'update_rate_hz': 20.0,
            'open_min_s': 8.0, 'open_max_s': 16.0,
            'closed_min_s': 5.0, 'closed_max_s': 11.0,
            'startup_delay_s': 5.0, 'seed': -1,
            'service_timeout_ms': 1200,
            'max_extension_limit': 0.65,  # supplied by simulation.launch.py
        }
        positions = [
            ('gate_1', 0.0, 1.5, 0.13, -math.pi / 2),
            ('gate_2', -1.2, 0.47, 0.13, 0.0),
            ('gate_3', 1.2, -0.16, 0.13, math.pi),
            ('gate_4', 0.25, -1.5, 0.13, math.pi / 2),
        ]
        for name, x, y, z, yaw in positions:
            defaults.update({f'{name}_x': x, f'{name}_y': y,
                             f'{name}_z': z, f'{name}_yaw': yaw,
                             f'{name}_extension': 0.65})
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        self.speed = float(p['speed_mps'])
        self.open_min, self.open_max = float(p['open_min_s']), float(p['open_max_s'])
        self.closed_min, self.closed_max = float(p['closed_min_s']), float(p['closed_max_s'])
        self.rate = float(p['update_rate_hz'])
        self.timeout_ms = int(p['service_timeout_ms'])
        self.service = f"/world/{p['world_name']}/set_pose_vector"
        self.rng = random.Random(None if int(p['seed']) == -1 else int(p['seed']))
        if not (0 < self.speed <= 0.2):
            raise ValueError('Expected gate speed in (0, 0.2] m/s')
        if not (0 < self.open_min <= self.open_max and
                0 < self.closed_min <= self.closed_max and 0 < self.rate <= 30):
            raise ValueError('Invalid timing or update frequency')

        extension_limit = float(p['max_extension_limit'])
        if not math.isfinite(extension_limit) or extension_limit <= 0.0:
            raise ValueError('max_extension_limit must be finite and positive')
        self.gates = []
        for name, _, _, _, _ in positions:
            extension = float(p[f'{name}_extension'])
            if not (0.0 < extension <= extension_limit):
                raise ValueError(
                    f'{name}_extension must be in (0, {extension_limit:g}] m')
            g = Gate(name, float(p[f'{name}_x']), float(p[f'{name}_y']),
                     float(p[f'{name}_z']), float(p[f'{name}_yaw']),
                     0.0, 0.0, 'open',
                     float(p['startup_delay_s']) + self.rng.uniform(self.open_min, self.open_max),
                     max_extension=extension)
            self.gates.append(g)
            self.get_logger().info(f'{name}: retracted; first closing in {g.remaining:.1f}s')

        self.last_ns = None
        self.pending = None
        self.last_error_ns = 0
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(
            f'Kinematic gates via {self.service}, speed={self.speed} m/s, '
            f'extensions={{' + ', '.join(f'{g.name}: {g.max_extension:.2f} m' for g in self.gates) + '}, ' +
            f'commands={self.rate} Hz (requested; actual limited by gz service latency)')

    def _request(self):
        # protobuf text format for gz.msgs.Pose_V (repeated Pose pose).
        pieces = []
        for g in self.gates:
            x = g.x0 + g.distance * math.cos(g.yaw)
            y = g.y0 + g.distance * math.sin(g.yaw)
            qw = math.cos(g.yaw / 2.0)
            qz = math.sin(g.yaw / 2.0)
            pieces.append(
                f'pose {{ name: "{g.name}" '
                f'position {{ x: {x:.10f} y: {y:.10f} z: {g.z0:.10f} }} '
                f'orientation {{ w: {qw:.10f} x: 0 y: 0 z: {qz:.10f} }} }}'
            )
        return ' '.join(pieces)

    def _publish_poses(self, now_ns):
        # Do not accumulate subprocesses if Gazebo is slow or unavailable.
        if self.pending is not None:
            status = self.pending.poll()
            if status is None:
                return
            if status != 0 and now_ns - self.last_error_ns > 5_000_000_000:
                self.get_logger().warn('Gazebo set_pose_vector call failed; check world service')
                self.last_error_ns = now_ns
            self.pending = None
        command = [
            'gz', 'service', '-s', self.service,
            '--reqtype', 'gz.msgs.Pose_V',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.timeout_ms),
            '--req', self._request(),
        ]
        try:
            self.pending = subprocess.Popen(
                command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError as exc:
            if now_ns - self.last_error_ns > 5_000_000_000:
                self.get_logger().error(f'Could not start Gazebo CLI: {exc}')
                self.last_error_ns = now_ns

    def tick(self):
        now_ns = self.get_clock().now().nanoseconds
        if self.last_ns is None or now_ns < self.last_ns:
            self.last_ns = now_ns
            return
        dt = max(0.0, min((now_ns - self.last_ns) * 1e-9, 0.4))
        self.last_ns = now_ns
        if dt == 0.0:
            return
        for g in self.gates:
            if g.state in ('open', 'closed'):
                g.remaining -= dt
                if g.remaining <= 0.0:
                    g.goal = g.max_extension if g.state == 'open' else 0.0
                    g.start_distance = g.distance
                    g.motion_elapsed = 0.0
                    length = abs(g.goal - g.start_distance)
                    # Cubic easing: max instantaneous velocity = 1.5 * length / duration.
                    # This keeps peak speed at or below speed_mps.
                    g.motion_duration = max(0.001, 1.5 * length / self.speed)
                    g.state = 'closing' if g.state == 'open' else 'opening'
                    self.get_logger().info(
                        f'{g.name}: {g.state.upper()} ({length:.2f} m in '
                        f'{g.motion_duration:.1f}s)')
            if g.state in ('opening', 'closing'):
                g.motion_elapsed += dt
                u = min(1.0, g.motion_elapsed / g.motion_duration)
                smooth = u * u * (3.0 - 2.0 * u)
                g.distance = g.start_distance + (g.goal - g.start_distance) * smooth
                if u >= 1.0:
                    g.distance = g.goal
                    g.state = 'open' if g.state == 'opening' else 'closed'
                    g.remaining = self.rng.uniform(
                        self.open_min, self.open_max) if g.state == 'open' else self.rng.uniform(
                        self.closed_min, self.closed_max)
                    self.get_logger().info(f'{g.name}: {g.state.upper()} for {g.remaining:.1f}s')
        self._publish_poses(now_ns)

    def destroy_node(self):
        if self.pending is not None and self.pending.poll() is None:
            self.pending.terminate()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KinematicRandomGates()
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
