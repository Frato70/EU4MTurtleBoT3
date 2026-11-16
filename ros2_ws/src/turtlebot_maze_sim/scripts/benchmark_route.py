#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped


# ============================================================
# CONFIGURACIÓN DEL CONTROLADOR
# ============================================================

# Velocidades máximas
MAX_LINEAR_SPEED = 0.08       # m/s
MAX_ANGULAR_SPEED = 0.40      # rad/s

# Ganancias
KP_LINEAR = 0.8
KP_ANGULAR = 2.0

# Distancia máxima al waypoint para considerarlo alcanzado
POSITION_TOLERANCE = 0.015    # 1.5 cm

# Si el error angular supera este valor:
# primero gira y NO avanza
ROTATE_ONLY_THRESHOLD = math.radians(8.0)

# Pequeña espera inicial, medida usando el timestamp de Gazebo
START_DELAY = 1.0


# ============================================================
# RUTA DEL EXPERIMENTO
# ============================================================
#
# ESTA ES LA ÚNICA PARTE QUE DEBES MODIFICAR PARA DISEÑAR
# LA TRAYECTORIA.
#
# Cada punto es:
#
#     (x, y)
#
# en metros respecto a la posición INICIAL del robot.
#
# Sistema local:
#
#             +Y
#              ↑
#              |
#              |
#              ● robot inicial ─────→ +X
#
#
# Estos puntos corresponden aproximadamente a la ruta que
# anteriormente habías definido mediante giros y distancias.
#
# IMPORTANTE:
# Ya NO defines giros.
# El robot calcula automáticamente qué orientación necesita
# para llegar al siguiente waypoint.
#

WAYPOINTS_LOCAL = [
    # Primera recta después del giro inicial
    (0.00, -0.91),
    # Segundo corredor
    (0.60, -0.91),
    # Subir
    (0.60, -0.57),
    # Izquierda
    (0.33, -0.57),
    # Subir
    (0.33, -0.30),
    # Derecha
    (0.60, -0.30),
    # Subir
    (0.60, 0.03),
    # izquierda
    (0.33, 0.03),
    # izquierda
    (0.33, 0.30),
    # izquierda
    (0.60, 0.30),
    # izquierda
    (0.60, 0.60),
    # izquierda
    (0.00, 0.60),
    # izquierda
    (0.00, 0.30),
    # izquierda
    (-0.30, 0.30),
    # izquierda
    (-0.30, 0.00),
    # izquierda
    (0.00, 0.00),
]


# ============================================================
# NO ES NECESARIO MODIFICAR NADA DEBAJO DE ESTA LÍNEA
# ============================================================


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def normalize_angle(angle):

    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(q):

    siny_cosp = 2.0 * (
        q.w * q.z
        + q.x * q.y
    )

    cosy_cosp = 1.0 - 2.0 * (
        q.y * q.y
        + q.z * q.z
    )

    return math.atan2(
        siny_cosp,
        cosy_cosp
    )


class BenchmarkRoute(Node):

    def __init__(self):

        super().__init__(
            "benchmark_route"
        )

        # ----------------------------------------------------
        # Publisher
        # ----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            "/cmd_vel",
            10,
        )

        # ----------------------------------------------------
        # Ground truth Gazebo
        # ----------------------------------------------------

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/ground_truth_pose",
            self.pose_callback,
            10,
        )

        # ----------------------------------------------------
        # Estado
        # ----------------------------------------------------

        self.initialized = False
        self.finished = False

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.start_time = None

        self.waypoints_world = []

        self.waypoint_index = 0

        self.get_logger().info(
            "Benchmark path follower ready."
        )

        self.get_logger().info(
            "Waiting for /ground_truth_pose..."
        )

    # ========================================================
    # Inicialización
    # ========================================================

    def initialize_route(self, msg):

        pose = msg.pose

        self.start_x = pose.position.x
        self.start_y = pose.position.y

        self.start_yaw = quaternion_to_yaw(
            pose.orientation
        )

        self.start_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )

        cos_yaw = math.cos(
            self.start_yaw
        )

        sin_yaw = math.sin(
            self.start_yaw
        )

        # ----------------------------------------------------
        # Transformar ruta local -> mundo Gazebo
        # ----------------------------------------------------

        for local_x, local_y in WAYPOINTS_LOCAL:

            world_x = (
                self.start_x
                + cos_yaw * local_x
                - sin_yaw * local_y
            )

            world_y = (
                self.start_y
                + sin_yaw * local_x
                + cos_yaw * local_y
            )

            self.waypoints_world.append(
                (world_x, world_y)
            )

        self.initialized = True

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "GROUND TRUTH ROUTE INITIALIZED"
        )

        self.get_logger().info(
            f"Start: "
            f"x={self.start_x:.3f}, "
            f"y={self.start_y:.3f}, "
            f"yaw={math.degrees(self.start_yaw):.2f} deg"
        )

        for index, waypoint in enumerate(
            self.waypoints_world
        ):

            self.get_logger().info(
                f"Waypoint {index + 1}: "
                f"x={waypoint[0]:.3f}, "
                f"y={waypoint[1]:.3f}"
            )

        self.get_logger().info(
            "======================================"
        )

    # ========================================================
    # Ground truth callback
    # ========================================================

    def pose_callback(self, msg):

        # ----------------------------------------------------
        # Primera pose
        # ----------------------------------------------------

        if not self.initialized:

            self.initialize_route(msg)
            self.publish_velocity(
                0.0,
                0.0,
                msg,
            )

            return

        # ----------------------------------------------------
        # Ya terminó
        # ----------------------------------------------------

        if self.finished:

            self.publish_velocity(
                0.0,
                0.0,
                msg,
            )

            return

        current_time = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )

        # ----------------------------------------------------
        # Espera inicial
        # ----------------------------------------------------

        if (
            current_time - self.start_time
            < START_DELAY
        ):

            self.publish_velocity(
                0.0,
                0.0,
                msg,
            )

            return

        # ----------------------------------------------------
        # Pose real actual
        # ----------------------------------------------------

        current_x = msg.pose.position.x
        current_y = msg.pose.position.y

        current_yaw = quaternion_to_yaw(
            msg.pose.orientation
        )

        # ----------------------------------------------------
        # Waypoint actual
        # ----------------------------------------------------

        target_x, target_y = (
            self.waypoints_world[
                self.waypoint_index
            ]
        )

        dx = target_x - current_x
        dy = target_y - current_y

        distance = math.hypot(
            dx,
            dy,
        )

        # ----------------------------------------------------
        # Waypoint alcanzado
        # ----------------------------------------------------

        if distance <= POSITION_TOLERANCE:

            self.publish_velocity(
                0.0,
                0.0,
                msg,
            )

            self.get_logger().info(
                f"Waypoint "
                f"{self.waypoint_index + 1}/"
                f"{len(self.waypoints_world)} "
                f"reached. "
                f"Error={distance:.4f} m"
            )

            self.waypoint_index += 1

            # ------------------------------------------------
            # Ruta completa
            # ------------------------------------------------

            if (
                self.waypoint_index
                >= len(self.waypoints_world)
            ):

                self.finished = True

                self.get_logger().info(
                    "======================================"
                )

                self.get_logger().info(
                    "BENCHMARK ROUTE FINISHED"
                )

                self.get_logger().info(
                    "Robot stopped."
                )

                self.get_logger().info(
                    "======================================"
                )

            return

        # ----------------------------------------------------
        # Dirección necesaria para llegar al waypoint
        # ----------------------------------------------------

        target_heading = math.atan2(
            dy,
            dx,
        )

        heading_error = normalize_angle(
            target_heading
            - current_yaw
        )

        # ----------------------------------------------------
        # Control angular
        # ----------------------------------------------------

        angular_velocity = clamp(
            KP_ANGULAR * heading_error,
            -MAX_ANGULAR_SPEED,
            MAX_ANGULAR_SPEED,
        )

        # ----------------------------------------------------
        # Si todavía está muy mal orientado:
        # GIRAR EN EL SITIO
        # ----------------------------------------------------

        if (
            abs(heading_error)
            > ROTATE_ONLY_THRESHOLD
        ):

            linear_velocity = 0.0

        else:

            # ------------------------------------------------
            # Control proporcional de distancia
            # ------------------------------------------------

            linear_velocity = min(
                MAX_LINEAR_SPEED,
                KP_LINEAR * distance,
            )

            # Reducir velocidad si todavía existe
            # pequeño error angular.
            heading_factor = max(
                0.0,
                math.cos(heading_error)
            )

            linear_velocity *= (
                heading_factor
            )

        # ----------------------------------------------------
        # Publicar cmd_vel
        # ----------------------------------------------------

        self.publish_velocity(
            linear_velocity,
            angular_velocity,
            msg,
        )

    # ========================================================
    # Publicación
    # ========================================================

    def publish_velocity(
        self,
        linear_x,
        angular_z,
        pose_msg,
    ):

        cmd = TwistStamped()

        cmd.header.stamp = (
            pose_msg.header.stamp
        )

        cmd.header.frame_id = (
            "base_footprint"
        )

        cmd.twist.linear.x = float(
            linear_x
        )

        cmd.twist.linear.y = 0.0
        cmd.twist.linear.z = 0.0

        cmd.twist.angular.x = 0.0
        cmd.twist.angular.y = 0.0

        cmd.twist.angular.z = float(
            angular_z
        )

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(
        args=args
    )

    node = BenchmarkRoute()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        # Intentar detener el robot al cerrar
        stop = TwistStamped()

        stop.twist.linear.x = 0.0
        stop.twist.angular.z = 0.0

        for _ in range(10):
            node.cmd_pub.publish(stop)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()