#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from pumas_vision_msgs.msg import VisionObject
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PointStamped
import tf2_ros
from tf2_ros import TransformException


HFOV = (86 * 3.14159265358979323846) / 180.0
VFOV = (57 * 3.14159265358979323846) / 180.0
HEAD_Z = 1.05


class GoalRobotPoseNode(Node):
    """
    Nodo que se suscribe a /vision/ball y /vision/goal_center,
    calcula un punto (x,y) alineado a 0.5m del balón respecto al robot
    y publica en /goal_robot_pose como VisionObject.
    El balón queda en medio entre el punto y el goal_center.
    Incluye proyección a coordenadas de imagen para visualización.
    """
 
    def __init__(self):
        super().__init__("goal_robot_pose")

        self.declare_parameter("distance_from_ball_m", 0.25)
        self.declare_parameter("ball_move_threshold_m", 0.3)

        self.distance_from_ball = self.get_parameter("distance_from_ball_m").value
        self.ball_move_threshold = self.get_parameter("ball_move_threshold_m").value

        self.ball_x = None
        self.ball_y = None
        self.goal_x = None
        self.goal_y = None
        self.head_pan = 0.0
        self.head_tilt = 0.0
        self.img_width = 640
        self.img_height = 480
        self.last_published_ball_odom_x = None
        self.last_published_ball_odom_y = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.sub_ball = self.create_subscription(
            VisionObject, "/vision/ball", self.callback_ball, 10
        )
        self.sub_goal = self.create_subscription(
            VisionObject, "/vision/goal_center", self.callback_goal, 10
        )
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self.callback_joints, 10
        )
        self.sub_img = self.create_subscription(
            Image, "/camera/color/image_raw", self.callback_img, 10
        )
        self.pub_point = self.create_publisher(
            VisionObject, "/goal_robot_pose", 10
        )

        self.declare_parameter("update_period_s", 3.0)
        period = self.get_parameter("update_period_s").value
        self.timer = self.create_timer(period, self.timer_callback)
    
    def callback_ball(self, msg: VisionObject):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y

    def callback_goal(self, msg: VisionObject):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def callback_joints(self, msg: JointState):
        self.head_pan = msg.position[0]
        self.head_tilt = msg.position[1]

    def callback_img(self, msg: Image):
        self.img_width = msg.width
        self.img_height = msg.height

    def _ball_in_odom(self):
        """Transforma posición del balón (base_link) a odom."""
        if self.ball_x is None or self.ball_y is None:
            return None, None
        try:
            p = PointStamped()
            p.header.frame_id = "base_link"
            p.header.stamp = self.get_clock().now().to_msg()
            p.point.x = float(self.ball_x)
            p.point.y = float(self.ball_y)
            p.point.z = 0.0
            p_odom = self.tf_buffer.transform(p, "odom")
            return p_odom.point.x, p_odom.point.y
        except (TransformException, Exception):
            return float(self.ball_x), float(self.ball_y)

    def _cartesian_to_image(self, x: float, y: float):
        """Proyecta punto (x,y) en frame robot a coordenadas de imagen."""
        L = math.sqrt(x * x + y * y + HEAD_Z * HEAD_Z)
        if L < 1e-6:
            return 0, 0
        theta = math.atan2(y, x)
        phi = math.asin(HEAD_Z / L)
        img_x = self.img_width / 2 - (theta - self.head_pan) * self.img_width / HFOV
        img_y = self.img_height / 2 + (phi - self.head_tilt) * self.img_height / VFOV
        return int(img_x), int(img_y)

    def timer_callback(self):
        if (
            self.ball_x is None
            or self.ball_y is None
            or self.goal_x is None
            or self.goal_y is None
        ):
            return

        # No actualizar si el balón no se ha movido más allá del umbral en odom
        # (evita actualizar por movimiento del robot; solo cuando el balón se mueve en el mundo)
        ball_odom_x, ball_odom_y = self._ball_in_odom()
        if (self.last_published_ball_odom_x is not None and self.last_published_ball_odom_y is not None
                and ball_odom_x is not None and ball_odom_y is not None):
            dx = ball_odom_x - self.last_published_ball_odom_x
            dy = ball_odom_y - self.last_published_ball_odom_y
            if math.hypot(dx, dy) < self.ball_move_threshold:
                return

        # Vector de balón hacia goal_center
        dx = self.goal_x - self.ball_x
        dy = self.goal_y - self.ball_y

        dist = math.sqrt(dx * dx + dy * dy)
        if dist < 1e-6:
            return

        # Dirección unitaria de balón hacia arco
        ux = dx / dist
        uy = dy / dist

        # Punto: 0.25m detrás del balón (opuesto al arco)
        # Balón queda en medio entre el punto y el goal_center
        point_x = self.ball_x - self.distance_from_ball * ux
        point_y = self.ball_y - self.distance_from_ball * uy

        if ball_odom_x is not None and ball_odom_y is not None:
            self.last_published_ball_odom_x = ball_odom_x
            self.last_published_ball_odom_y = ball_odom_y

        img_x, img_y = self._cartesian_to_image(point_x, point_y)

        msg = VisionObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.id = "goal_robot_pose"
        msg.confidence = 1.0
        msg.x = img_x
        msg.y = img_y
        msg.width = 10
        msg.height = 10
        msg.pose.position.x = point_x
        msg.pose.position.y = point_y
        msg.pose.position.z = 0.0

        self.pub_point.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GoalRobotPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
