#!/usr/bin/env python3

"""
Nodo que lleva al robot a la posición goal_robot_pose con dirección hacia la portería.
Inspirado en ball_follower y move_to_ball.
"""

import math
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pumas_vision_msgs.msg import VisionObject
import tf2_ros
from tf2_ros import TransformException


def normalize_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def shortest_angular_distance(fr: float, to: float) -> float:
    return normalize_angle(to - fr)


def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class State(Enum):
    Waiting = 0
    Center = 1
    Approach = 2
    Face_Goal = 3
    Finished = 4


class GoToGoalPoseNode(Node):
    """
    Nodo que navega al punto goal_robot_pose y se orienta hacia la portería.
    Similar a ball_follower/move_to_ball: publica Twist a /cmd_vel.
    """

    def __init__(self):
        super().__init__("go_to_goal_pose")

        self.declare_parameter("enable_topic", "/go_to_goal_pose/enable")
        self.declare_parameter("finish_topic", "/go_to_goal_pose/finish")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("position_tol_m", 0.08)
        self.declare_parameter("center_tol_y_m", 0.06)
        self.declare_parameter("yaw_tol_rad", 0.15)
        self.declare_parameter("kp_center_y", 1.2)
        self.declare_parameter("kp_forward", 0.8)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_lin_x", 0.3)
        self.declare_parameter("max_lin_y", 0.25)
        self.declare_parameter("max_ang_z", 0.9)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.finish_pub = self.create_publisher(
            Bool, self.get_parameter("finish_topic").value, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, self.get_parameter("enable_topic").value, self.callback_enable, 10
        )
        self.sub_goal_pose = self.create_subscription(
            VisionObject, "/goal_robot_pose", self.callback_goal_pose, 10
        )
        self.sub_goal_center = self.create_subscription(
            VisionObject, "/vision/goal_center", self.callback_goal_center, 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.enabled = False
        self.finish_sent = False
        self.current_state = State.Waiting

        self.target_x = None
        self.target_y = None
        self.goal_x = None
        self.goal_y = None
        self.get_logger().info("go_to_goal_pose node started, waiting for enable")

    def callback_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.enabled = True
            self.finish_sent = False
            self.current_state = State.Center
            self.get_logger().info("Enabled -> Center")
        elif not msg.data and self.enabled:
            self.enabled = False
            self.current_state = State.Waiting
            self.finish_sent = False
            self._publish_zero()
            self.get_logger().info("Disabled -> Waiting")

    def callback_goal_pose(self, msg: VisionObject):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

    def callback_goal_center(self, msg: VisionObject):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def _get_base_yaw(self):
        try:
            t = self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            )
            return normalize_angle(yaw_from_quaternion(t.transform.rotation))
        except TransformException:
            return None

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def timer_callback(self):
        if not self.enabled:
            return

        if self.current_state == State.Finished:
            self._publish_zero()
            if not self.finish_sent:
                m = Bool()
                m.data = True
                self.finish_pub.publish(m)
                self.finish_sent = True
                self.get_logger().info("go_to_goal_pose finished")
            return

        # Usar últimos datos publicados; si nunca hubo datos, esperar
        if self.target_x is None or self.target_y is None:
            self._publish_zero()
            return

        tx = float(self.target_x)
        ty = float(self.target_y)
        r = math.hypot(tx, ty)

        pos_tol = self.get_parameter("position_tol_m").value
        center_tol = self.get_parameter("center_tol_y_m").value
        kp_center = self.get_parameter("kp_center_y").value
        kp_fwd = self.get_parameter("kp_forward").value
        kp_yaw = self.get_parameter("kp_yaw").value
        max_lin_x = self.get_parameter("max_lin_x").value
        max_lin_y = self.get_parameter("max_lin_y").value
        max_ang_z = self.get_parameter("max_ang_z").value
        yaw_tol = self.get_parameter("yaw_tol_rad").value

        twist = Twist()

        if self.current_state == State.Center:
            if abs(ty) > center_tol:
                twist.linear.y = clamp(kp_center * ty, -max_lin_y, max_lin_y)
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.current_state = State.Approach
                self.get_logger().info("Centered -> Approach")
            return

        if self.current_state == State.Approach:
            if r > pos_tol:
                vx = clamp(kp_fwd * r, 0.05, max_lin_x)
                vy = clamp(kp_center * ty, -max_lin_y, max_lin_y)
                twist.linear.x = vx
                twist.linear.y = vy
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.current_state = State.Face_Goal
                self.get_logger().info("Reached position -> Face_Goal")
            return

        if self.current_state == State.Face_Goal:
            if self.goal_x is None or self.goal_y is None:
                self._publish_zero()
                return

            target_yaw = math.atan2(self.goal_y, self.goal_x)
            yaw = self._get_base_yaw()
            if yaw is None:
                self._publish_zero()
                return

            yaw_err = shortest_angular_distance(yaw, target_yaw)
            if abs(yaw_err) > yaw_tol:
                twist.angular.z = clamp(kp_yaw * yaw_err, -max_ang_z, max_ang_z)
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.current_state = State.Finished
                self.get_logger().info("Facing goal -> Finished")
            return


def main(args=None):
    rclpy.init(args=args)
    node = GoToGoalPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
