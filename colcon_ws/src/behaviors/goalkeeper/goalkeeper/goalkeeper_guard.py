import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import Bool
from pumas_vision_msgs.msg import VisionObject

from goalkeeper.KalmanBallTracker import KalmanBallTracker
from goalkeeper.PIDController import PIDController


SM_INIT = 0
SM_SEARCH_BALL = 2
SM_TRACK_BALL = 3
SM_DEFEND = 4
SM_INTERCEPT = 7
SM_CLEAR_BALL = 8
SM_RECOVER = 9


class GoalkeeperGuard(Node):

    def __init__(self):
        super().__init__("goalkeeper_guard")

        self.get_logger().info(
            "INITIALIZING GOALKEEPER GUARD NODE"
        )

        # Activación de la máquina de estados

        self.enable = False
        self.restart_state_machine = False

        self.measured_x = 0.0
        self.measured_y = 0.0

        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0


        self.predicted_goal_y = 0.0
        self.t_cross = 0.0
        self.has_ball = False
        self.has_prediction = False

        # El robot se encuentra sobre la línea x = 0
        self.goal_x = 0.0

        # Rango lateral que puede cubrir el portero
        self.min_goal_y = -1.0
        self.max_goal_y = 1.0

        # Rango de tiempo aceptable para la predicción
        self.min_prediction_time = 0.05
        self.max_prediction_time = 2.0

 
        self.last_ball_time = self.get_clock().now()

        # Cámara de 10 FPS: cada medición llega aproximadamente
        # cada 0.1 segundos.
        self.ball_timeout = 0.35

        # Control lateral
        self.target_y = 0.0
        self.control_mode = "search"
        self.goal_y_tolerance = 0.07

        self.kalman = KalmanBallTracker(dt=0.1)

        self.pid_y = PIDController(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            output_min=-0.5,
            output_max=0.5
        )

        self.last_control_time = self.get_clock().now()


        self.current_head_pan = 0.0
        self.current_head_tilt = 0.0


        self.pub_cmd_vel = self.create_publisher(
            Twist,
            "/cmd_vel",
            1
        )

        self.pub_sgn_enable = self.create_publisher(
            Bool,
            "/planning/head_ball_follower/enable",
            1
        )

        # --------------------------------------------------
        # Subscribers
        # --------------------------------------------------
        self.sub_enable = self.create_subscription(
            Bool,
            "/goalkeeper_SM/enable",
            self.callback_enable,
            1
        )

        self.sub_ball = self.create_subscription(
            VisionObject,
            "/vision/ball",
            self.callback_ball,
            1
        )

        self.sub_joints = self.create_subscription(
            JointState,
            "/joint_states",
            self.callback_joint_states,
            1
        )

        self.get_logger().info(
            "Waiting for Bool message on /goalkeeper_SM/enable"
        )



    def callback_enable(self, msg):
        previous_enable = self.enable
        self.enable = bool(msg.data)

        # Flanco False -> True
        if self.enable and not previous_enable:
            self.restart_state_machine = True

            self.has_prediction = False
            self.pid_y.reset()
            self.last_control_time = self.get_clock().now()

            head_enable = Bool()
            head_enable.data = True
            self.pub_sgn_enable.publish(head_enable)

            self.get_logger().info(
                "Goalkeeper state machine ENABLED"
            )

        # Flanco True -> False
        elif not self.enable and previous_enable:
            self.restart_state_machine = False

            self.has_prediction = False
            self.pid_y.reset()
            self.stop_robot()

            head_enable = Bool()
            head_enable.data = False
            self.pub_sgn_enable.publish(head_enable)

            self.get_logger().info(
                "Goalkeeper state machine DISABLED"
            )



    def callback_joint_states(self, msg):
        if len(msg.position) < 2:
            return

        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]

    def callback_ball(self, msg):
        self.last_ball_time = self.get_clock().now()

        # Datos crudos de visión
        self.measured_x = float(msg.pose.position.x)
        self.measured_y = float(msg.pose.position.y)

        # Actualizar Kalman incluso si la máquina está desactivada.
        # Así el filtro tendrá datos recientes cuando sea habilitada.
        estimated_state = self.kalman.step(
            self.measured_x,
            self.measured_y
        )

        self.ball_x = estimated_state["x"]
        self.ball_y = estimated_state["y"]
        self.ball_vx = estimated_state["vx"]
        self.ball_vy = estimated_state["vy"]

        self.has_ball = True

        prediction = self.predict_goal_crossing()

        if prediction is not None:
            self.predicted_goal_y, self.t_cross = prediction
            self.has_prediction = True

            self.get_logger().info(
                f"Kalman: "
                f"x={self.ball_x:.3f}, "
                f"y={self.ball_y:.3f}, "
                f"vx={self.ball_vx:.3f}, "
                f"vy={self.ball_vy:.3f}, "
                f"y_cross={self.predicted_goal_y:.3f}, "
                f"t_cross={self.t_cross:.3f}"
            )
        else:
            self.has_prediction = False
            self.t_cross = 0.0

    # ======================================================
    # PREDICCIÓN DE CRUCE
    # ======================================================

    def predict_goal_crossing(self):
        """
        Calcula dónde cruzará la pelota la línea de portería.

        Se supone:
        - El robot está sobre la línea x = 0.
        - La pelota está delante del robot con x > 0.
        - vx < 0 significa que se acerca a la portería.
        """

        x = self.ball_x
        y = self.ball_y
        vx = self.ball_vx
        vy = self.ball_vy

        # No predecir si la pelota no se acerca.
        if vx >= -0.001:
            return None

        # Evitar división entre cero.
        if abs(vx) < 0.001:
            return None

        t_cross = (self.goal_x - x) / vx

        if t_cross <= self.min_prediction_time:
            return None

        if t_cross > self.max_prediction_time:
            return None

        predicted_y = y + vy * t_cross

        predicted_y = max(
            self.min_goal_y,
            min(self.max_goal_y, predicted_y)
        )

        return predicted_y, t_cross



    def control_goalkeeper_y(self):
        now = self.get_clock().now()

        dt = (
            now - self.last_control_time
        ).nanoseconds / 1e9

        self.last_control_time = now

        if dt <= 0.0:
            return

        if not self.has_ball:
            self.pid_y.reset()
            self.stop_robot()
            self.control_mode = "search"
            return

        if self.has_prediction:
            # La pelota va hacia la portería:
            # usar el punto futuro calculado con Kalman.
            self.target_y = self.predicted_goal_y
            self.control_mode = "defend"

        else:
            # No hay predicción:

            self.target_y = self.measured_y
            self.control_mode = "track"

        error_y = self.target_y

        if abs(error_y) < self.goal_y_tolerance:
            self.pid_y.reset()
            self.stop_robot()

            self.get_logger().info(
                f"PID STOP [{self.control_mode}]: "
                f"target_y={self.target_y:.3f}, "
                f"error_y={error_y:.3f}, "
                f"tolerance={self.goal_y_tolerance:.3f}"
            )
            return

        control_y = self.pid_y.update(error_y, dt)

        cmd = Twist()
        cmd.linear.y = float(control_y)

        self.pub_cmd_vel.publish(cmd)

        self.get_logger().info(
            f"PID [{self.control_mode}]: "
            f"target_y={self.target_y:.3f}, "
            f"error_y={error_y:.3f}, "
            f"cmd_y={control_y:.3f}"
        )

    # ======================================================
    # FUNCIONES AUXILIARES
    # ======================================================

    def update_ball_timeout(self):
        now = self.get_clock().now()

        elapsed = (
            now - self.last_ball_time
        ).nanoseconds / 1e9

        if elapsed > self.ball_timeout:
            self.has_ball = False
            self.has_prediction = False
            self.t_cross = 0.0

            self.pid_y.reset()
            self.stop_robot()

    def stop_robot(self):
        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.pub_cmd_vel.publish(cmd)

    def is_ball_threatening_goal(self):
        if not self.has_prediction:
            return False

        return (
            self.min_goal_y
            <= self.predicted_goal_y
            <= self.max_goal_y
        )

    def get_single_image(self, timeout_seconds=1.0):
        self.get_logger().info("Waiting for single image")

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        success, msg = wait_for_message(
            msg_type=Image,
            node=self,
            topic="/camera/color/image_raw",
            qos_profile=qos_profile,
            time_to_wait=timeout_seconds
        )

        return msg if success else None

    # ======================================================
    # MÁQUINA DE ESTADOS
    # ======================================================

    def spin(self):
        self.get_logger().info(
            "Goalkeeper node ready. "
            "Waiting for /goalkeeper_SM/enable"
        )

        state = SM_INIT

        while rclpy.ok():

            # Debe ejecutarse aun cuando la máquina esté deshabilitada,
            # para poder recibir el Bool de activación.
            rclpy.spin_once(
                self,
                timeout_sec=0.01
            )

            # Al recibir un nuevo True, reiniciar en SM_INIT.
            if self.restart_state_machine:
                state = SM_INIT
                self.restart_state_machine = False

            # Máquina desactivada.
            if not self.enable:
                self.stop_robot()
                time.sleep(0.02)
                continue

            self.update_ball_timeout()

            if state == SM_INIT:
                self.get_logger().info("SM_INIT")

                head_enable = Bool()
                head_enable.data = True
                self.pub_sgn_enable.publish(head_enable)

                self.stop_robot()
                self.pid_y.reset()
                self.has_prediction = False
                self.last_control_time = self.get_clock().now()

                state = SM_SEARCH_BALL

            elif state == SM_SEARCH_BALL:
                self.get_logger().info("SM_SEARCH_BALL")

                self.stop_robot()

                if self.has_ball:
                    state = SM_TRACK_BALL

            elif state == SM_TRACK_BALL:
                self.get_logger().info("SM_TRACK_BALL")

                if not self.has_ball:
                    self.stop_robot()
                    self.pid_y.reset()

                    state = SM_SEARCH_BALL

                else:
                    self.control_goalkeeper_y()

                    if self.has_prediction:
                        state = SM_DEFEND
                    else:
                        state = SM_TRACK_BALL

            elif state == SM_DEFEND:
                self.get_logger().info("SM_DEFEND")

                if not self.has_ball:
                    self.stop_robot()
                    self.pid_y.reset()

                    state = SM_SEARCH_BALL

                else:
                    self.control_goalkeeper_y()

                    if self.has_prediction:
                        state = SM_DEFEND
                    else:
                        state = SM_TRACK_BALL

            elif state == SM_RECOVER:
                self.get_logger().info("SM_RECOVER")

                self.stop_robot()
                self.pid_y.reset()
                self.has_prediction = False

                state = SM_SEARCH_BALL

            else:
                self.get_logger().warning(
                    f"Unknown state: {state}. Returning to SM_INIT."
                )
                state = SM_INIT

            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)

    node = GoalkeeperGuard()

    try:
        node.spin()

    except KeyboardInterrupt:
        node.get_logger().info(
            "Goalkeeper node interrupted"
        )

    finally:
        node.stop_robot()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()