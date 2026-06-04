import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Image
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, Bool
from rclpy.wait_for_message import wait_for_message
from booster_interface.srv import RpcService
from pumas_vision_msgs.msg import VisionObject
from goalkeeper.KalmanBallTracker import KalmanBallTracker
from goalkeeper.PIDController import PIDController

SM_INIT = 0

SM_SEARCH_BALL = 2  # El robot no ve la pelota
SM_TRACK_BALL = 3   # El robot ya detectó la pelota. Ahora: sigue su posición, estima velocidad, mantiene alineación.
SM_DEFEND = 4       # Elrobot se mueve rápidamente, cubre el ángulo y bloquea la trayectoria, 
SM_INTERCEPT = 7    # El robot sale hacia el balón
SM_CLEAR_BALL = 8   # El robot despeja la pelota: Manda la pelota: lejos o hacia un compañero
SM_RECOVER =  9     # El robot perdió estabilidad o quedó fuera de posición, debe: regresar al centro, reorientarse,estabilizar sensores.

class GoalkeeperGuard(Node):
    def callback_joint_states(self, msg):
        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]



    def callback_ball(self, msg):
        measured_x = msg.pose.position.x
        measured_y = msg.pose.position.y

        self.measured_x = measured_x
        self.measured_y = measured_y

        state = self.kalman.step(measured_x, measured_y)

        self.ball_x = state["x"]
        self.ball_y = state["y"]
        self.ball_vx = state["vx"]
        self.ball_vy = state["vy"]

        self.has_ball = True

        prediction = self.predict_goal_crossing()

        if prediction is not None:
            self.predicted_goal_y, self.t_cross = prediction
            self.has_prediction = True

            self.get_logger().info(
                f"Kalman: x={self.ball_x:.3f}, y={self.ball_y:.3f}, "
                f"vx={self.ball_vx:.3f}, vy={self.ball_vy:.3f}, "
                f"y_cross={self.predicted_goal_y:.3f}, "
                f"t_cross={self.t_cross:.3f}"
            )
        else:
            self.has_prediction = False



    def predict_goal_crossing(self):
        # Predice en qué y cruzará la pelota la línea de portería. Como el robot está sobre la línea de portería, la portería está en x = 0 respecto al robot.

        x = self.ball_x
        y = self.ball_y
        vx = self.ball_vx
        vy = self.ball_vy

        # Si vx >= 0, la pelota no viene hacia el robot/portería
        if vx >= -0.001:
            return None

        # Tiempo para llegar al plano x = 0
        t_cross = -x / vx

        if t_cross <= self.min_prediction_time:
            return None

        if t_cross > self.max_prediction_time:
            return None

        predicted_y = y + vy * t_cross

        # Limitar al rango físico que puede cubrir el portero
        predicted_y = max(self.min_goal_y, min(self.max_goal_y, predicted_y))

        return predicted_y, t_cross



    def control_goalkeeper_y(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now

        if not self.has_ball or not self.has_prediction:
            self.pid_y.reset()
            self.stop_robot()
            return

        # Como el robot está en y = 0 de su propio marco,
        # el error es directamente el y donde cruzará la pelota.
        error_y = self.predicted_goal_y

        control_y = self.pid_y.update(error_y, dt)

        cmd = Twist()

        # Si el robot se mueve al lado contrario, cambia por -control_y
        cmd.linear.y = float(control_y)

        #self.pub_cmd_vel.publish(cmd)
        self.get_logger().info("Se publica velocidad")

        self.get_logger().info(
            f"PID: target_y={self.predicted_goal_y:.3f}, "
            f"error_y={error_y:.3f}, "
            f"cmd_y={control_y:.3f}"
        )


    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd)


    def is_ball_threatening_goal(self):
        if not self.has_prediction:
            return False

        # Si y_cross está dentro del rango de la portería,
        # entonces sí puede ser amenaza.
        if self.predicted_goal_y < self.min_goal_y:
            return False

        if self.predicted_goal_y > self.max_goal_y:
            return False

        return True


    def __init__(self):
        super().__init__("goalkeeper_guard")
        self.get_logger().info("INITIALIZING GOAL KEEEPER GUARD NODE - ")
        self.measured_x = 0
        self.measured_y = 0
        self.ball_x = 0.0
        self.ball_y = 0.0
        self.ball_vx = 0.0
        self.ball_vy = 0.0
        self.predicted_goal_y = 0.0
        self.t_cross = 0.0
        self.located_ball_flag = False
        self.has_ball = False
        self.has_prediction = False
        # Como el robot está sobre la línea de portería, la portería está en x = 0
        self.goal_x = 0.0

        # Ajusta estos valores al ancho real que puede cubrir el robot
        self.min_goal_y = -0.6
        self.max_goal_y = 0.6

        # Rango de tiempo razonable para hacer la predicción
        self.min_prediction_time = 0.05
        self.max_prediction_time = 2.0

        self.move = 0.0
        self.current_head_pan  = 0
        self.current_head_tilt = 0
        self.center_x = 640
        self.center_y = 360
        self.tolerance = 0
        self.kalman = KalmanBallTracker(dt=0.033)
        self.pid_y = PIDController(
            kp=1.0,
            ki=0.0,
            kd=0.1,
            output_min=-0.35,
            output_max=0.35
        )
        self.last_control_time = self.get_clock().now()
        self.sub_ball = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.pub_sgn_enable = self.create_publisher(Bool, "/planning/head_ball_follower/enable", 1)
        self.sub_joints  = self.create_subscription(JointState, "/joint_states", self.callback_joint_states, 1)
        

    def get_single_image(self, timeout_seconds=1):
        self.get_logger().info("Waiting for single image")
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        success, msg = wait_for_message(
            msg_type=Image,
            node=self,
            topic="/camera/color/image_raw",
            qos_profile=qos_profile,
            time_to_wait=timeout_seconds
        )
        return msg if success else None

    def spin(self):
        self.get_logger().info("INITIALIZING GOALKEEPER NODE - ")

        state = SM_INIT

        while rclpy.ok():

            if state == SM_INIT:
                self.get_logger().info("SM_INIT")

                msg = Bool()
                msg.data = True
                self.pub_sgn_enable.publish(msg)

                self.stop_robot()
                self.pid_y.reset()

                state = SM_SEARCH_BALL

            elif state == SM_SEARCH_BALL:
                self.get_logger().info("SM_SEARCH_BALL")

                self.stop_robot()

                if self.has_ball:
                    state = SM_TRACK_BALL
                else:
                    state = SM_SEARCH_BALL

            elif state == SM_TRACK_BALL:
                self.get_logger().info("SM_TRACK_BALL")

                # En este estado ya hay pelota, pero puede o no venir hacia portería.
                if not self.has_ball:
                    self.stop_robot()
                    self.pid_y.reset()
                    state = SM_SEARCH_BALL

                elif self.has_prediction:
                    state = SM_DEFEND

                else:
                    # Hay pelota, pero no hay trayectoria peligrosa o no se puede predecir.
                    self.stop_robot()
                    state = SM_TRACK_BALL

            elif state == SM_DEFEND:
                self.get_logger().info("SM_DEFEND")

                if not self.has_ball:
                    self.stop_robot()
                    self.pid_y.reset()
                    state = SM_SEARCH_BALL

                elif not self.has_prediction:
                    self.stop_robot()
                    self.pid_y.reset()
                    state = SM_TRACK_BALL

                else:
                    # Aquí se aplica el PID
                    self.control_goalkeeper_y()
                    state = SM_TRACK_BALL

            elif state == SM_RECOVER:
                self.get_logger().info("SM_RECOVER")

                self.stop_robot()
                self.pid_y.reset()
                self.has_prediction = False

                state = SM_SEARCH_BALL

            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.02)



def main(args=None):
    rclpy.init(args=args)
    node = GoalkeeperGuard()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()