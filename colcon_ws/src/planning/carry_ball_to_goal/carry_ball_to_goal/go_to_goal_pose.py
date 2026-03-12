#!/usr/bin/env python3

"""
Nodo que lleva al robot a la posición goal_robot_pose con dirección hacia la portería.
Orquesta: navegación (Center -> Approach -> Face_Goal), búsqueda de portería cuando
se pierde, y reanudación cuando se encuentra de nuevo.
Inspirado en ball_follower y move_to_ball.
"""

import math
import rclpy
from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
from sensor_msgs.msg import JointState
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
    Following = 1      # Center -> Approach -> Face_Goal
    GoalLost = 2      # Usa cache; si balón se mueve mucho -> SearchingGoal
    SearchingGoal = 3  # Busca portería con giros de cabeza
    Finished = 4
    Center = 10       # Sub-estados de Following
    Approach = 11
    Face_Goal = 12


# Poses de cabeza para buscar la portería
LOOK_FOR_GOAL_POSES = [
    [0.0, 0.7], [-0.8, 0.7], [-0.8, 0.2], [0.0, 0.2], [0.8, 0.2], [0.8, 0.7]
]


class GoToGoalPoseNode(Node):
    """
    Nodo que navega al punto goal_robot_pose y se orienta hacia la portería.
    Cuando pierde la portería: usa último goal_robot_pose válido; si el balón
    se mueve mucho, busca la portería con giros de cabeza.
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
        self.declare_parameter("ball_move_threshold_m", 0.3)
        self.declare_parameter("goal_timeout_s", 2.0)
        self.declare_parameter("search_pose_hold_s", 2.0)

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.finish_pub = self.create_publisher(
            Bool, self.get_parameter("finish_topic").value, 10
        )
        self.pub_head = self.create_publisher(
            Float32MultiArray, "/hardware/head/goal_pose", 10
        )

        self.enable_sub = self.create_subscription(
            Bool, self.get_parameter("enable_topic").value, self._callback_enable, 10
        )
        self.sub_goal_pose = self.create_subscription(
            VisionObject, "/goal_robot_pose", self._callback_goal_pose, 10
        )
        self.sub_goal_center = self.create_subscription(
            VisionObject, "/vision/goal_center", self._callback_goal_center, 10
        )
        self.sub_ball = self.create_subscription(
            VisionObject, "/vision/ball", self._callback_ball, 10
        )
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self._callback_joints, 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self._timer_callback)

        self.enabled = False
        self.finish_sent = False
        self.current_state = State.Waiting
        self.sub_state = State.Center  # Sub-estado dentro de Following

        self.target_x = None
        self.target_y = None
        self.cached_target_x = None
        self.cached_target_y = None
        self.goal_x = None
        self.goal_y = None
        self.last_goal_time = None
        self.ball_x = None
        self.ball_y = None
        self.last_ball_x = None
        self.last_ball_y = None
        self.img_ball_x = None
        self.img_ball_y = None
        self.img_width = 640
        self.img_height = 480
        self.img_goal_x = 320
        self.img_goal_y = 240
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.goal_pan = 0.0
        self.goal_tilt = 0.0
        self.look_poses = list(LOOK_FOR_GOAL_POSES)
        self.last_search_pose_time = None

        self.get_logger().info("go_to_goal_pose node started, waiting for enable")

    def _callback_enable(self, msg: Bool):
        if msg.data and not self.enabled:
            self.enabled = True
            self.finish_sent = False
            self.current_state = State.Following
            self.sub_state = State.Center
            self.look_poses = list(LOOK_FOR_GOAL_POSES)
            self.goal_pan = self.current_pan
            self.goal_tilt = self.current_tilt
            self.get_logger().info("Enabled -> Following")
        elif not msg.data and self.enabled:
            self.enabled = False
            self.current_state = State.Waiting
            self.finish_sent = False
            self._publish_zero()
            self.get_logger().info("Disabled -> Waiting")

    def _callback_goal_pose(self, msg: VisionObject):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.cached_target_x = msg.pose.position.x
        self.cached_target_y = msg.pose.position.y

    def _callback_goal_center(self, msg: VisionObject):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.last_goal_time = self.get_clock().now()

    def _callback_ball(self, msg: VisionObject):
        self.last_ball_x = self.ball_x
        self.last_ball_y = self.ball_y
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y
        self.img_ball_x = msg.x
        self.img_ball_y = msg.y
        # Actualizar head para mirar al balón (como head_ball_follower)
        if self.img_ball_x is not None and self.img_ball_y is not None:
            error_x = -(self.img_ball_x - self.img_goal_x) / (self.img_width / 2)
            error_y = (self.img_ball_y - self.img_goal_y) / (self.img_height / 2)
            self.goal_pan += 0.15 * error_x
            self.goal_tilt += 0.15 * error_y
            self.goal_pan = max(-1.0, min(1.0, self.goal_pan))
            self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))

    def _callback_joints(self, msg: JointState):
        self.current_pan = msg.position[0]
        self.current_tilt = msg.position[1]

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

    def _goal_is_fresh(self) -> bool:
        if self.last_goal_time is None:
            return False
        timeout = self.get_parameter("goal_timeout_s").value
        age = (self.get_clock().now() - self.last_goal_time).nanoseconds * 1e-9
        return age <= timeout

    def _ball_moved_significantly(self) -> bool:
        if self.ball_x is None or self.ball_y is None:
            return False
        if self.last_ball_x is None or self.last_ball_y is None:
            return False
        dx = self.ball_x - self.last_ball_x
        dy = self.ball_y - self.last_ball_y
        return math.hypot(dx, dy) >= self.get_parameter("ball_move_threshold_m").value

    def _get_target_for_navigation(self):
        if self.target_x is not None and self.target_y is not None:
            return self.target_x, self.target_y
        return self.cached_target_x, self.cached_target_y

    def _publish_head_look_at_ball(self):
        """Publica pose de cabeza para mirar al balón (como head_ball_follower)."""
        if self.img_ball_x is None or self.img_ball_y is None:
            return
        msg = Float32MultiArray()
        msg.data = [self.goal_pan, self.goal_tilt]
        self.pub_head.publish(msg)

    def _timer_callback(self):
        if not self.enabled:
            return

        if self.current_state == State.Waiting:
            return

        if self.current_state == State.Finished:
            self._publish_zero()
            if not self.finish_sent:
                self.finish_pub.publish(Bool(data=True))
                self.finish_sent = True
                self.get_logger().info("go_to_goal_pose finished")
            return

        if self.current_state == State.SearchingGoal:
            self._do_searching_goal()
            return

        if self.current_state == State.GoalLost:
            self._do_goal_lost()
            self._publish_head_look_at_ball()
            return

        self._do_following()
        self._publish_head_look_at_ball()

    def _do_searching_goal(self):
        if self._goal_is_fresh():
            self.current_state = State.Following
            self.sub_state = State.Center
            self.goal_pan = self.current_pan
            self.goal_tilt = self.current_tilt
            self.get_logger().info("Goal found -> Following")
            return

        hold_s = self.get_parameter("search_pose_hold_s").value
        now = self.get_clock().now()
        if self.last_search_pose_time is not None:
            elapsed = (now - self.last_search_pose_time).nanoseconds * 1e-9
            if elapsed < hold_s:
                return

        self._publish_zero()
        if not self.look_poses:
            self.look_poses = list(LOOK_FOR_GOAL_POSES)

        head_pose = self.look_poses.pop(0)
        self.look_poses.append(head_pose)
        self.get_logger().info(f"Searching for goal at head ({head_pose[0]:.2f}, {head_pose[1]:.2f})")

        msg = Float32MultiArray()
        msg.data = head_pose
        self.pub_head.publish(msg)
        self.last_search_pose_time = now

    def _do_goal_lost(self):
        if self._goal_is_fresh():
            self.current_state = State.Following
            self.sub_state = State.Center
            self.get_logger().info("Goal visible again -> Following")
            return

        if self._ball_moved_significantly():
            self.current_state = State.SearchingGoal
            self.look_poses = list(LOOK_FOR_GOAL_POSES)
            self.get_logger().info("Ball moved a lot -> SearchingGoal")
            return

        tx, ty = self._get_target_for_navigation()
        if tx is None or ty is None:
            self._publish_zero()
            return

        r = math.hypot(tx, ty)
        pos_tol = self.get_parameter("position_tol_m").value
        if r <= pos_tol:
            self._publish_zero()
            return

        kp_fwd = self.get_parameter("kp_forward").value
        kp_center = self.get_parameter("kp_center_y").value
        max_lin_x = self.get_parameter("max_lin_x").value
        max_lin_y = self.get_parameter("max_lin_y").value

        twist = Twist()
        twist.linear.x = clamp(kp_fwd * r, 0.05, max_lin_x)
        twist.linear.y = clamp(kp_center * ty, -max_lin_y, max_lin_y)
        self.cmd_pub.publish(twist)

    def _do_following(self):
        if not self._goal_is_fresh():
            self.current_state = State.GoalLost
            self.get_logger().info("Goal timeout -> GoalLost")
            return

        tx, ty = self._get_target_for_navigation()
        if tx is None or ty is None:
            self._publish_zero()
            return

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

        if self.sub_state == State.Center:
            if abs(ty) > center_tol:
                twist.linear.y = clamp(kp_center * ty, -max_lin_y, max_lin_y)
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.sub_state = State.Approach
                self.get_logger().info("Centered -> Approach")
            return

        if self.sub_state == State.Approach:
            if r > pos_tol:
                vx = clamp(kp_fwd * r, 0.05, max_lin_x)
                vy = clamp(kp_center * ty, -max_lin_y, max_lin_y)
                twist.linear.x = vx
                twist.linear.y = vy
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.sub_state = State.Face_Goal
                self.get_logger().info("Reached position -> Face_Goal")
            return

        if self.sub_state == State.Face_Goal:
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
