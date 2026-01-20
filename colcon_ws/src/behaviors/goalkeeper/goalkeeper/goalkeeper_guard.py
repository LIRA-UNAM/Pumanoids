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

SM_INIT = 0
SM_WAIT_BALL_CENTER = 1
SM_ERROR_CALCULATION = 5
SM_BALL_TRACKING = 6

class GoalkeeperGuard(Node):
    def callback_joint_states(self, msg):
        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]

    def callback_ball(self, msg):
        self.ball_center_x = msg.x
        self.ball_center_y = msg.pose.position.y

    def __init__(self):
        super().__init__("goalkeeper_guard")
        self.get_logger().info("INITIALIZING GOAL KEEEPER GUARD NODE - ")
        self.ball_center_x = 0
        self.ball_center_y = 0
        self.move = 0.0
        self.current_head_pan  = 0
        self.current_head_tilt = 0
        self.center_x = 640
        self.center_y = 360
        self.tolerance = 0
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
            #if self.enable:
            if 1 :
                if state == SM_INIT:
                    self.get_logger().info("Initializing state machine for goalkepeer............")
                    self.get_logger().info("Sending enable signal............")
                    msg = Bool()
                    msg.data = True
                    self.pub_sgn_enable.publish(msg)
                    state = SM_WAIT_BALL_CENTER


                elif state == SM_WAIT_BALL_CENTER:
                    self.get_logger().info("SM_WAIT_BALL_CENTER")
                    self.get_logger().info("ball center: ")
                    self.get_logger().info(str(self.ball_center_y))



                    state = SM_ERROR_CALCULATION


                elif state == SM_ERROR_CALCULATION:
                    self.get_logger().info("SM_WAIT_BALL_CENTER")

                    if self.ball_center_y > 0.02:
                        self.get_logger().info("mueve a la izquierda")
                        time.sleep(0.5)
                        self.move = 0.2

                    elif self.ball_center_y < -0.02:
                        self.get_logger().info("mueve a la derecha")
                        time.sleep(0.5)
                        self.move = -0.2
                        
                    else:
                        self.get_logger().info("sin movimiento")
                        time.sleep(0.5)
                        self.move = 0.0

                    state = SM_BALL_TRACKING


                elif state == SM_BALL_TRACKING:
                    self.get_logger().info("SM_BALL_TRACKING")
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.y = float(self.move)
                    self.pub_cmd_vel.publish(cmd_vel_msg)
                    state = SM_INIT
            #
            # END
            #
            else:
                state = SM_INIT

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