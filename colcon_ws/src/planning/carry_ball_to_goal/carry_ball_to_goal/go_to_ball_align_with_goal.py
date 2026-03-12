#!/usr/bin/env python3

"""
Nodo go_to_ball_align_with_goal:
1. Busca el balón con poses de cabeza.
2. Una vez encontrado, va hacia él (Center -> Approach).
3. Cuando está a 0.25 m del balón, se detiene.
4. Busca el goal center con poses de cabeza.
5. Se alinea moviendo de lado: robot - balón - goal (linear.y según error de bearing).
6. Publica finish.
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


class State(Enum):
    Waiting = 0
    SearchingBall = 1
    GoingToBall = 2
    Center = 10
    Approach = 11
    SearchingGoal = 3
    Aligning = 4   # Mover de lado para alinear robot-balón-goal
    Finished = 5


LOOK_POSES = [
    [0.0, 0.7], [-0.8, 0.7], [-0.8, 0.2], [0.0, 0.2], [0.8, 0.2], [0.8, 0.7]
]


class GoToBallAlignWithGoalNode(Node):
    """
    Busca balón -> va hacia él -> al llegar (0.25 m) busca goal -> alinea lateralmente.
    """

    def __init__(self):
        super().__init__("go_to_ball_align_with_goal")

        self.declare_parameter("enable_topic", "/go_to_ball_align_with_goal/enable")
        self.declare_parameter("finish_topic", "/go_to_ball_align_with_goal/finish")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("arrival_distance_m", 0.25)
        self.declare_parameter("center_tol_y_m", 0.06)
        self.declare_parameter("align_bearing_tol_rad", 0.05)
        self.declare_parameter("kp_center_y", 1.2)
        self.declare_parameter("kp_forward", 0.8)
        self.declare_parameter("max_lin_x", 0.3)
        self.declare_parameter("max_lin_y", 0.25)
        self.declare_parameter("ball_timeout_s", 0.5)
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

        # Activación por booleano comentada: se ejecuta en nodo aparte, cancelar detiene
        # self.enable_sub = self.create_subscription(
        #     Bool, self.get_parameter("enable_topic").value, self._callback_enable, 10
        # )
        self.sub_ball = self.create_subscription(
            VisionObject, "/vision/ball", self._callback_ball, 10
        )
        self.sub_goal = self.create_subscription(
            VisionObject, "/vision/goal_center", self._callback_goal, 10
        )
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self._callback_joints, 10
        )

        self.timer = self.create_timer(0.05, self._timer_callback)

        self.enabled = True  # Sin activación por topic; cancelar nodo detiene
        self.finish_sent = False
        self.current_state = State.SearchingBall
        self.sub_state = State.Center

        self.ball_x = None
        self.ball_y = None
        self.last_ball_time = None
        self.img_ball_x = None
        self.img_ball_y = None
        self.img_goal_x = 320
        self.img_goal_y = 240
        self.img_width = 640
        self.img_height = 480

        self.goal_x = None
        self.goal_y = None
        self.last_goal_time = None

        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.goal_pan = 0.0
        self.goal_tilt = 0.0

        self.look_poses = list(LOOK_POSES)
        self.last_search_pose_time = None

        self.get_logger().info("go_to_ball_align_with_goal started (no enable topic)")

    # def _callback_enable(self, msg: Bool):
    #     if msg.data and not self.enabled:
    #         self.enabled = True
    #         ...
    #     elif not msg.data and self.enabled:
    #         self.enabled = False
    #         ...

    def _callback_ball(self, msg: VisionObject):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y
        self.img_ball_x = msg.x
        self.img_ball_y = msg.y
        self.last_ball_time = self.get_clock().now()
        if self.current_state in (State.GoingToBall, State.SearchingBall):
            if self.img_ball_x is not None and self.img_ball_y is not None:
                error_x = -(msg.x - self.img_goal_x) / (self.img_width / 2)
                error_y = (msg.y - self.img_goal_y) / (self.img_height / 2)
                self.goal_pan += 0.15 * error_x
                self.goal_tilt += 0.15 * error_y
                self.goal_pan = max(-1.0, min(1.0, self.goal_pan))
                self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))

    def _callback_goal(self, msg: VisionObject):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.last_goal_time = self.get_clock().now()
        if self.current_state in (State.SearchingGoal, State.Aligning):
            if hasattr(msg, 'x') and hasattr(msg, 'y') and msg.x is not None:
                error_x = -(msg.x - self.img_goal_x) / (self.img_width / 2)
                error_y = (msg.y - self.img_goal_y) / (self.img_height / 2)
                self.goal_pan += 0.15 * error_x
                self.goal_tilt += 0.15 * error_y
                self.goal_pan = max(-1.0, min(1.0, self.goal_pan))
                self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))

    def _callback_joints(self, msg: JointState):
        self.current_pan = msg.position[0]
        self.current_tilt = msg.position[1]

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _ball_is_fresh(self) -> bool:
        if self.last_ball_time is None:
            return False
        timeout = self.get_parameter("ball_timeout_s").value
        age = (self.get_clock().now() - self.last_ball_time).nanoseconds * 1e-9
        return age <= timeout

    def _goal_is_fresh(self) -> bool:
        if self.last_goal_time is None:
            return False
        timeout = self.get_parameter("goal_timeout_s").value
        age = (self.get_clock().now() - self.last_goal_time).nanoseconds * 1e-9
        return age <= timeout

    def _publish_head_ball(self):
        if self.img_ball_x is not None and self.img_ball_y is not None:
            msg = Float32MultiArray()
            msg.data = [self.goal_pan, self.goal_tilt]
            self.pub_head.publish(msg)

    def _publish_head_goal(self):
        if self.goal_x is not None and self.goal_y is not None:
            msg = Float32MultiArray()
            msg.data = [self.goal_pan, self.goal_tilt]
            self.pub_head.publish(msg)

    def _publish_head_search_poses(self):
        if not self.look_poses:
            self.look_poses = list(LOOK_POSES)
        hold_s = self.get_parameter("search_pose_hold_s").value
        now = self.get_clock().now()
        if self.last_search_pose_time is not None:
            elapsed = (now - self.last_search_pose_time).nanoseconds * 1e-9
            if elapsed < hold_s:
                return
        head_pose = self.look_poses.pop(0)
        self.look_poses.append(head_pose)
        msg = Float32MultiArray()
        msg.data = head_pose
        self.pub_head.publish(msg)
        self.last_search_pose_time = now

    def _timer_callback(self):
        if not self.enabled:
            return
        # if self.current_state == State.Waiting:
        #     return
        if self.current_state == State.Finished:
            self._publish_zero()
            if not self.finish_sent:
                self.finish_pub.publish(Bool(data=True))
                self.finish_sent = True
                self.get_logger().info("go_to_ball_align_with_goal finished")
            return
        if self.current_state == State.SearchingBall:
            self._do_searching_ball()
            return
        if self.current_state == State.GoingToBall:
            self._do_going_to_ball()
            return
        if self.current_state == State.SearchingGoal:
            self._do_searching_goal()
            return
        if self.current_state == State.Aligning:
            self._do_aligning()
            return

    def _do_searching_ball(self):
        if self._ball_is_fresh() and self.ball_x is not None and self.ball_y is not None:
            self.current_state = State.GoingToBall
            self.sub_state = State.Center
            self.goal_pan = self.current_pan
            self.goal_tilt = self.current_tilt
            self.get_logger().info("Ball found -> GoingToBall")
            return
        self._publish_zero()
        self._publish_head_search_poses()

    def _do_going_to_ball(self):
        if not self._ball_is_fresh():
            self.current_state = State.SearchingBall
            self.look_poses = list(LOOK_POSES)
            self.get_logger().info("Ball lost -> SearchingBall")
            self._publish_head_search_poses()
            return
        if self.ball_x is None or self.ball_y is None:
            self._publish_zero()
            return

        r = math.hypot(self.ball_x, self.ball_y)
        arrival_dist = self.get_parameter("arrival_distance_m").value
        center_tol = self.get_parameter("center_tol_y_m").value
        kp_center = self.get_parameter("kp_center_y").value
        kp_fwd = self.get_parameter("kp_forward").value
        max_lin_x = self.get_parameter("max_lin_x").value
        max_lin_y = self.get_parameter("max_lin_y").value

        self._publish_head_ball()
        twist = Twist()

        if self.sub_state == State.Center:
            if abs(self.ball_y) > center_tol:
                twist.linear.y = clamp(kp_center * self.ball_y, -max_lin_y, max_lin_y)
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.sub_state = State.Approach
                self.get_logger().info("Centered -> Approach")
            return

        if self.sub_state == State.Approach:
            if r > arrival_dist:
                vx = clamp(kp_fwd * (r - arrival_dist), 0.05, max_lin_x)
                vy = clamp(kp_center * self.ball_y, -max_lin_y, max_lin_y)
                twist.linear.x = vx
                twist.linear.y = vy
                self.cmd_pub.publish(twist)
            else:
                self._publish_zero()
                self.current_state = State.SearchingGoal
                self.look_poses = list(LOOK_POSES)
                self.get_logger().info(f"Arrived at ball (r={r:.2f}m) -> SearchingGoal")
            return

    def _do_searching_goal(self):
        if self._goal_is_fresh() and self.goal_x is not None and self.goal_y is not None:
            self.current_state = State.Aligning
            self.get_logger().info("Goal found -> Aligning")
            return
        self._publish_zero()
        self._publish_head_search_poses()

    def _do_aligning(self):
        """Alineación lateral: mover de lado para que robot-balón-goal queden alineados."""
        if not self._goal_is_fresh():
            self.current_state = State.SearchingGoal
            self.look_poses = list(LOOK_POSES)
            self.get_logger().info("Goal lost -> SearchingGoal")
            self._publish_head_search_poses()
            return

        if self.ball_x is None or self.ball_y is None or self.goal_x is None or self.goal_y is None:
            self._publish_zero()
            self._publish_head_goal()
            return

        bearing_ball = math.atan2(self.ball_y, self.ball_x)
        bearing_goal = math.atan2(self.goal_y, self.goal_x)
        bearing_err = shortest_angular_distance(bearing_ball, bearing_goal)

        tol = self.get_parameter("align_bearing_tol_rad").value
        if abs(bearing_err) < tol:
            self._publish_zero()
            self.current_state = State.Finished
            self.get_logger().info("Aligned -> Finished")
            return

        kp_center = self.get_parameter("kp_center_y").value
        max_lin_y = self.get_parameter("max_lin_y").value
        twist = Twist()
        twist.linear.y = clamp(kp_center * bearing_err, -max_lin_y, max_lin_y)
        self.cmd_pub.publish(twist)
        self._publish_head_goal()


def main(args=None):
    rclpy.init(args=args)
    node = GoToBallAlignWithGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
