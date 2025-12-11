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
SM_WAIT_FOR_FIRST_IMAGE = 1
SM_LOOK_AT_BALL = 2
SM_LOOK_FOR_BALL = 3
SM_GOAL_KEEPER_GUARD = 4

class GoalkeeperGuard(Node):
    def callback_enable(self, msg):
        self.enable = msg.data
        if self.enable:
            self.get_logger().info("Enable received...")
        else:
            self.get_logger().info("Disable received...")

    def callback_joint_states(self, msg):
        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]

    def callback_ball(self, msg):
        self.ball_center_x = msg.x
        self.ball_center_y = msg.y

    def __init__(self):
        super().__init__("goalkeeper_guard")
        self.get_logger().info("INITIALIZING GOAL KEEEPER GUARD NODE - ")
        self.enable = False
        self.current_head_pan  = 0
        self.current_head_tilt = 0
        self.sub_enable  = self.create_subscription(Bool, "/behaviors/goalkeeper_guard/enable", self.callback_enable, 1)
        self.sub_ball = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
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
        self.get_logger().info("Waiting for enable signal")
        state = SM_INIT
        no_new_data_counter = 0
        while rclpy.ok():
            if self.enable:
                if state == SM_INIT:
                    self.get_logger().info("Initializing state machine for head ball follower...")
                    state = SM_WAIT_FOR_FIRST_IMAGE

                elif state == SM_WAIT_FOR_FIRST_IMAGE:
                    img = self.get_single_image()
                    if img is not None:
                        self.get_logger().info(f"Image received with size {img.width}x{img.height}")
                        self.img_width  = img.width
                        self.img_height = img.height
                        self.img_goal_x = img.width/2
                        self.img_goal_y = img.height/2
                        state = -1
                    else:
                        None

                elif state == SM_LOOK_FOR_BALL:
                    self.get_logger().info("SM_LOOK_FOR_BALL")
                    if not self.new_ball_data:
                        head_pose = self.look_for_poses.pop(0)
                        self.look_for_poses.append(head_pose)
                        self.get_logger().info(f"Looking for ball at ({head_pose[0]},{head_pose[1]})")
                        pantilt_msg = Float32MultiArray()
                        pantilt_msg.data = head_pose
                        self.pub_pantilt.publish(pantilt_msg)
                        time.sleep(0.5)
                        rclpy.spin_once(self, timeout_sec=0.5)
                    else:
                        self.get_logger().info(f"Found ball at position ({self.img_ball_x},{self.img_ball_y}) with head at ({self.current_pan},{self.current_tilt})")
                        self.goal_pan  = -0.65*(self.img_ball_x - self.img_goal_x) / (self.img_width  / 2) + self.current_pan
                        self.goal_tilt =  0.50*(self.img_ball_y - self.img_goal_y) / (self.img_height / 2) + self.current_tilt
                        self.goal_pan  = max(-1.0, min(1.0, self.goal_pan))
                        self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))
                        pantilt_msg = Float32MultiArray()
                        pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                        self.pub_pantilt.publish(pantilt_msg)
                        state = SM_LOOK_AT_BALL

                elif state == SM_LOOK_AT_BALL:
                    self.get_logger().info("SM_LOOK_AT_BALL")
                    if self.new_ball_data:
                        self.new_ball_data = False
                        no_new_data_counter = 0
                        pantilt_msg = Float32MultiArray()
                        pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                        self.pub_pantilt.publish(pantilt_msg)
                        state = SM_GOAL_KEEPER_GUARD

                    else:
                        no_new_data_counter += 1
                        if no_new_data_counter > 30:
                            self.get_logger().info("Lost ball********")
                            no_new_data_counter = 0
                            state = SM_LOOK_FOR_BALL

                elif state == SM_GOAL_KEEPER_GUARD:
		    self.get_looger().info("SM_GOAL_KEEEPER_GUARD")
                    center_x = 640
                    #print(f"Ball detected at x: {ball_center_x}, y: {ball_center_y}")
                    error_x = (-self.ball_center_x + center_x)/640
                    if error_x < 0:
                        error_x = -math.sqrt(-error_x)
			self.get_logger().info("error_x < 0 = " + error_x)
                    else:
                        error_x = math.sqrt(error_x)
			self.get_logger().info("error_x > 0 = " + error_x)
                    
                    cmd_vel_msg = Twist()
                    cmd_vel_msg.linear.x = 0.2
                    #cmd_vel_msg.angular.z = 1.2 * (error_x + self.current_head_pan)
                    #self.pub_cmd_vel.publish(cmd_vel_msg)
            #
            # END OF IF ENABLE
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
