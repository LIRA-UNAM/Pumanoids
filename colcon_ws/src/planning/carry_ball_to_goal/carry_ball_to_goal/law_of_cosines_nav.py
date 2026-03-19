#!/usr/bin/env python3

"""
Nodo law_of_cosines_nav: usa ley de los cosenos con vértices
  A = goal_center, B = robot, C = goal_robot_pose.

1. Fija x,y de goal_robot_pose para el ciclo.
2. Calcula: ángulo de giro (β en B), distancia a caminar (a = BC), ángulo de alineación (γ en C).
3. Gira β, avanza distancia a, gira γ para quedar alineado con balón y goal.
4. Al llegar: busca balón y goal_center, verifica alineación (umbral).
5. Si alineado: finish. Si no: repite ley de cosenos.
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


def transform_point_to_frame(x: float, y: float, z: float, t) -> tuple:
    """Transforma punto de base_link a odom usando t (odom<-base_link)."""
    q = t.transform.rotation
    tx = t.transform.translation.x
    ty = t.transform.translation.y
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    x_odom = c * x - s * y + tx
    y_odom = s * x + c * y + ty
    return x_odom, y_odom


class State(Enum):
    Waiting = 0
    Searching = 1      # Buscar balón y goal para tener goal_robot_pose
    Planning = 2      # Calcular ley de cosenos, fijar target
    TurnToTarget = 3  # Girar β para mirar goal_robot_pose
    Walk = 4          # Avanzar distancia a
    TurnToGoal = 5    # Girar γ para alinearse con goal
    CheckAlign = 6    # Verificar si está alineado
    Finished = 7


LOOK_POSES = [
    [0.0, 0.7], [-0.8, 0.7], [-0.8, 0.2], [0.0, 0.2], [0.8, 0.2], [0.8, 0.7]
]


class LawOfCosinesNavNode(Node):
    """
    Triángulo: A=goal_center, B=robot, C=goal_robot_pose.
    Ley de cosenos: a=BC, b=AC, c=AB.
    β=ángulo en B, γ=ángulo en C.
    """

    def __init__(self):
        super().__init__("law_of_cosines_nav")

        self.declare_parameter("enable_topic", "/law_of_cosines_nav/enable")
        self.declare_parameter("finish_topic", "/law_of_cosines_nav/finish")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("position_tol_m", 0.08)
        self.declare_parameter("yaw_tol_rad", 0.12)
        self.declare_parameter("align_bearing_tol_rad", 0.06)
        self.declare_parameter("kp_forward", 0.8)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("max_lin_x", 0.3)
        self.declare_parameter("max_ang_z", 0.9)
        self.declare_parameter("turn_step_ang_vel", 0.8)
        self.declare_parameter("turn_step_time_min", 0.3)
        self.declare_parameter("walk_step_time", 0.8)
        self.declare_parameter("walk_step_vel", 0.18)
        self.declare_parameter("ball_timeout_s", 0.5)
        self.declare_parameter("goal_timeout_s", 2.0)
        self.declare_parameter("search_pose_hold_s", 2.0)
        self.declare_parameter("walk_fraction", 1.0)  # Fracción de a a caminar (0-1); al terminar recalcula β, γ, a

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
        self.sub_goal_pose = self.create_subscription(
            VisionObject, "/goal_robot_pose", self._callback_goal_pose, 10
        )
        self.sub_joints = self.create_subscription(
            JointState, "/joint_states", self._callback_joints, 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self._timer_callback)

        self.enabled = True  # Sin activación por topic; cancelar nodo detiene
        self.finish_sent = False
        self.current_state = State.Searching

        self.ball_x = None
        self.ball_y = None
        self.last_ball_time = None
        self.goal_x = None
        self.goal_y = None
        self.last_goal_time = None
        self.goal_pose_x = None
        self.goal_pose_y = None
        self.last_goal_pose_time = None

        self.img_goal_x = 320
        self.img_goal_y = 240
        self.img_width = 640
        self.img_height = 480
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.goal_pan = 0.0
        self.goal_tilt = 0.0
        self.look_poses = list(LOOK_POSES)
        self.last_search_pose_time = None

        # Target fijado para el ciclo (en odom)
        self.target_odom_x = None
        self.target_odom_y = None
        self.distance_to_walk = None
        self.walk_distance = None
        self.walk_start_time = None
        self.walk_duration = None
        self.turn_start_time = None
        self.turn_duration = None
        self.turn_angular_z = 0.0
        self.turn_angle_1 = None   # β: giro para mirar goal_robot_pose
        self.turn_angle_2 = None    # γ: giro al llegar para alinear con goal

        self.get_logger().info("law_of_cosines_nav started (no enable topic)")

    # def _callback_enable(self, msg: Bool):
    #     ...

    def _callback_ball(self, msg: VisionObject):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y
        self.last_ball_time = self.get_clock().now()
        if self.current_state in (State.Searching, State.CheckAlign):
            if msg.x is not None and msg.y is not None:
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
        if self.current_state in (State.Searching, State.CheckAlign):
            if hasattr(msg, 'x') and msg.x is not None:
                error_x = -(msg.x - self.img_goal_x) / (self.img_width / 2)
                error_y = (msg.y - self.img_goal_y) / (self.img_height / 2)
                self.goal_pan += 0.15 * error_x
                self.goal_tilt += 0.15 * error_y
                self.goal_pan = max(-1.0, min(1.0, self.goal_pan))
                self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))

    def _callback_goal_pose(self, msg: VisionObject):
        self.goal_pose_x = msg.pose.position.x
        self.goal_pose_y = msg.pose.position.y
        self.last_goal_pose_time = self.get_clock().now()

    def _callback_joints(self, msg: JointState):
        self.current_pan = msg.position[0]
        self.current_tilt = msg.position[1]

    def _get_tf_odom_base(self):
        try:
            return self.tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            )
        except TransformException:
            return None

    def _get_robot_yaw(self):
        t = self._get_tf_odom_base()
        if t is None:
            return None
        return normalize_angle(yaw_from_quaternion(t.transform.rotation))

    def _publish_zero(self):
        self.cmd_pub.publish(Twist())

    def _ball_is_fresh(self) -> bool:
        if self.last_ball_time is None:
            return False
        age = (self.get_clock().now() - self.last_ball_time).nanoseconds * 1e-9
        return age <= self.get_parameter("ball_timeout_s").value

    def _goal_is_fresh(self) -> bool:
        if self.last_goal_time is None:
            return False
        age = (self.get_clock().now() - self.last_goal_time).nanoseconds * 1e-9
        return age <= self.get_parameter("goal_timeout_s").value

    def _goal_pose_is_fresh(self) -> bool:
        if self.last_goal_pose_time is None:
            return False
        age = (self.get_clock().now() - self.last_goal_pose_time).nanoseconds * 1e-9
        return age <= 1.0

    def _publish_head_search(self):
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

    def _publish_head_at_goal(self):
        msg = Float32MultiArray()
        msg.data = [self.goal_pan, self.goal_tilt]
        self.pub_head.publish(msg)

    def _compute_triangle_and_plan(self) -> bool:
        """Calcula ley de cosenos. Retorna True si OK."""
        t = self._get_tf_odom_base()
        if t is None:
            return False
        if self.goal_x is None or self.goal_y is None or self.goal_pose_x is None or self.goal_pose_y is None:
            return False

        # Transformar a odom
        bx = t.transform.translation.x
        by = t.transform.translation.y
        ax, ay = transform_point_to_frame(self.goal_x, self.goal_y, 0.0, t)
        cx, cy = transform_point_to_frame(self.goal_pose_x, self.goal_pose_y, 0.0, t)

        # Triángulo: A=goal_center, B=robot, C=goal_robot_pose
        # a = BC, b = AC, c = AB
        a = math.hypot(cx - bx, cy - by)
        b = math.hypot(ax - cx, ay - cy)
        c = math.hypot(ax - bx, ay - by)

        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            return False

        # Ley de cosenos: cos(β) = (a² + c² - b²) / (2ac), β en B
        cos_beta = (a * a + c * c - b * b) / (2.0 * a * c)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)

        # cos(γ) = (a² + b² - c²) / (2ab), γ en C
        cos_gamma = (a * a + b * b - c * c) / (2.0 * a * b)
        cos_gamma = max(-1.0, min(1.0, cos_gamma))
        gamma = math.acos(cos_gamma)

        # Signo de β: cross product (C-B) × (A-B). Si > 0, C a la izquierda
        cross = (cx - bx) * (ay - by) - (cy - by) * (ax - bx)
        sign_beta = 1.0 if cross > 0 else -1.0

        # Ángulo para girar hacia C desde B
        desired_yaw = math.atan2(cy - by, cx - bx)
        current_yaw = self._get_robot_yaw()
        if current_yaw is None:
            return False
        turn_1 = shortest_angular_distance(current_yaw, desired_yaw)

        # Ángulo para girar en C hacia A (alineación)
        yaw_at_c_to_b = math.atan2(by - cy, bx - cx)
        yaw_at_c_to_a = math.atan2(ay - cy, ax - cx)
        turn_2 = shortest_angular_distance(yaw_at_c_to_b, yaw_at_c_to_a)

        frac = self.get_parameter("walk_fraction").value
        frac = max(0.01, min(1.0, frac))
        walk_a = a * frac

        self.target_odom_x = cx
        self.target_odom_y = cy
        self.distance_to_walk = a
        self.walk_distance = walk_a
        self.turn_angle_1 = turn_1
        self.turn_angle_2 = turn_2

        self.get_logger().info(
            f"Plan: a={a:.2f}m, walk={walk_a:.2f}m (frac={frac:.2f}), β={math.degrees(turn_1):.1f}°, γ={math.degrees(turn_2):.1f}°"
        )
        return True

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
                self.get_logger().info("law_of_cosines_nav finished")
            return

        if self.current_state == State.Searching:
            self._do_searching()
            return
        if self.current_state == State.Planning:
            self._do_planning()
            return
        if self.current_state == State.TurnToTarget:
            self._do_turn_to_target()
            return
        if self.current_state == State.Walk:
            self._do_walk()
            return
        if self.current_state == State.TurnToGoal:
            self._do_turn_to_goal()
            return
        if self.current_state == State.CheckAlign:
            self._do_check_align()
            return

    def _do_searching(self):
        if (self._ball_is_fresh() and self._goal_is_fresh() and self._goal_pose_is_fresh()
                and self.ball_x is not None and self.goal_x is not None
                and self.goal_pose_x is not None):
            self.current_state = State.Planning
            self.get_logger().info("Data ready -> Planning")
            return
        self._publish_zero()
        self._publish_head_search()

    def _do_planning(self):
        if self._compute_triangle_and_plan():
            self.current_state = State.TurnToTarget
            self.get_logger().info("Planning done -> TurnToTarget")
        else:
            self.current_state = State.Searching
            self.look_poses = list(LOOK_POSES)
            self.get_logger().warn("Planning failed -> Searching")

    def _do_turn_to_target(self):
        if self.turn_angle_1 is None:
            self._publish_zero()
            self.current_state = State.Searching
            return

        max_ang_z = self.get_parameter("max_ang_z").value
        turn_speed = self.get_parameter("turn_step_ang_vel").value
        desired_ang = self.turn_angle_1

        if abs(desired_ang) < self.get_parameter("yaw_tol_rad").value:
            self._publish_zero()
            self.turn_start_time = None
            self.turn_duration = None
            self.current_state = State.Walk
            self.get_logger().info("Turn beta was negligible -> Walk")
            return

        if self.turn_start_time is None:
            sign = 1.0 if desired_ang > 0 else -1.0
            self.turn_angular_z = clamp(sign * turn_speed, -max_ang_z, max_ang_z)
            self.turn_duration = max(abs(desired_ang) / abs(self.turn_angular_z), self.get_parameter("turn_step_time_min").value)
            self.turn_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Turn β open-loop: ang_vel={self.turn_angular_z:.3f}rad/s dur={self.turn_duration:.3f}s"
            )

        elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9
        if elapsed >= self.turn_duration:
            self._publish_zero()
            self.turn_start_time = None
            self.turn_duration = None
            self.current_state = State.Walk
            self.get_logger().info("Beta turn done -> Walk")
            return

        twist = Twist()
        twist.angular.z = self.turn_angular_z
        self.cmd_pub.publish(twist)

    def _do_walk(self):
        if self.walk_distance is None or self.walk_distance < 1e-6:
            self._publish_zero()
            self.current_state = State.TurnToGoal
            return

        walk_step_vel = self.get_parameter("walk_step_vel").value
        max_lin_x = self.get_parameter("max_lin_x").value

        if self.walk_start_time is None:
            self.walk_duration = max(self.walk_distance / walk_step_vel, 0.01)
            self.walk_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Walk open-loop: vel={min(walk_step_vel,max_lin_x):.3f}m/s dur={self.walk_duration:.3f}s"
            )

        elapsed = (self.get_clock().now() - self.walk_start_time).nanoseconds * 1e-9
        if elapsed >= self.walk_duration:
            self._publish_zero()
            self.walk_start_time = None
            self.walk_duration = None
            self.current_state = State.TurnToGoal
            self.get_logger().info("Walk step done -> TurnToGoal")
            return

        twist = Twist()
        twist.linear.x = clamp(walk_step_vel, 0.0, max_lin_x)
        self.cmd_pub.publish(twist)

    def _do_turn_to_goal(self):
        if self.turn_angle_2 is None:
            self._publish_zero()
            self.current_state = State.Searching
            return

        max_ang_z = self.get_parameter("max_ang_z").value
        turn_speed = self.get_parameter("turn_step_ang_vel").value
        desired_ang = self.turn_angle_2

        if abs(desired_ang) < self.get_parameter("yaw_tol_rad").value:
            self._publish_zero()
            self.turn_start_time = None
            self.turn_duration = None
            self.current_state = State.CheckAlign
            self.get_logger().info("Gamma negligible -> CheckAlign")
            return

        if self.turn_start_time is None:
            sign = 1.0 if desired_ang > 0 else -1.0
            self.turn_angular_z = clamp(sign * turn_speed, -max_ang_z, max_ang_z)
            self.turn_duration = max(abs(desired_ang) / abs(self.turn_angular_z), self.get_parameter("turn_step_time_min").value)
            self.turn_start_time = self.get_clock().now()
            self.get_logger().info(
                f"Turn γ open-loop: ang_vel={self.turn_angular_z:.3f}rad/s dur={self.turn_duration:.3f}s"
            )

        elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9
        if elapsed >= self.turn_duration:
            self._publish_zero()
            self.turn_start_time = None
            self.turn_duration = None
            self.current_state = State.CheckAlign
            self.get_logger().info("Gamma turn done -> CheckAlign (buscar balón/goal, recalcular β γ a)")
            return

        twist = Twist()
        twist.angular.z = self.turn_angular_z
        self.cmd_pub.publish(twist)

    def _do_check_align(self):
        if not self._ball_is_fresh() or not self._goal_is_fresh():
            self._publish_zero()
            self.current_state = State.Searching
            self.look_poses = list(LOOK_POSES)
            self.get_logger().info("CheckAlign: sin balón/goal -> Searching (recalcular β γ a)")
            return
        if self.ball_x is None or self.ball_y is None or self.goal_x is None or self.goal_y is None:
            self._publish_zero()
            self._publish_head_search()
            return

        bearing_ball = math.atan2(self.ball_y, self.ball_x)
        bearing_goal = math.atan2(self.goal_y, self.goal_x)
        bearing_err = abs(shortest_angular_distance(bearing_ball, bearing_goal))
        tol = self.get_parameter("align_bearing_tol_rad").value

        self._publish_head_at_goal()

        if bearing_err < tol:
            self._publish_zero()
            self.current_state = State.Finished
            self.get_logger().info("Aligned (checked) -> Finished")
            return

        self._publish_zero()
        self.target_odom_x = None
        self.target_odom_y = None
        self.current_state = State.Searching
        self.look_poses = list(LOOK_POSES)
        self.get_logger().info(f"Not aligned (err={math.degrees(bearing_err):.1f}°) -> Searching, re-apply law of cosines")


def main(args=None):
    rclpy.init(args=args)
    node = LawOfCosinesNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
