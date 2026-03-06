import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from pumas_vision_msgs.msg import VisionObject
from sensor_msgs.msg import Image

SM_INIT = 0
SM_WAIT_FOR_FIRST_IMAGE = 10
SM_LOOK_FOR_BALL = 20
SM_LOOK_AT_BALL = 30

class HeadBallFollowerNode(Node):
    def callback_enable(self, msg):
        self.enable = msg.data
        if self.enable:
            self.get_logger().info("Enable received...")
        else:
            self.get_logger().info("Disable received...")

    def callback_joint_states(self, msg):
        self.current_pan = msg.position[0]
        self.current_tilt = msg.position[1]
            
    def callback_ball(self, msg):
        self.img_ball_x = msg.x
        self.img_ball_y = msg.y
        error_x = -(msg.x - self.img_goal_x) / (self.img_width  / 2)
        error_y =  (msg.y - self.img_goal_y) / (self.img_height / 2)        
        self.goal_pan += 0.15 * error_x
        self.goal_tilt += 0.15 * error_y
        self.goal_pan  = max(-1.0, min(1.0, self.goal_pan))
        self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))
        self.new_ball_data = True

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
        
    def __init__(self):
        super().__init__("head_ball_follower")
        self.get_logger().info("INITIALIZING HEAD BALL FOLLOWER NODE - ")
        self.enable = False
        self.goal_pan = 0.0
        self.goal_tilt = 0.0
        self.new_ball_data = False
        self.img_width  = 640
        self.img_height = 480
        self.img_goal_x = 320
        self.img_goal_y = 240
        self.img_ball_x = 320
        self.img_ball_y = 240
        self.current_pan  = 0
        self.current_tilt = 0
        self.look_for_poses = [[0.0,0.7], [-0.8, 0.7], [-0.8, 0.2], [0.0, 0.2], [0.8, 0.2], [0.8,0.7]]

        # Declare timestamp variable
        self.last_image_time = rclpy.time.Time(nanoseconds=0, clock_type=self.get_clock().clock_type)
        self.sub_enable  = self.create_subscription(Bool, "/head_ball_follower/enable", self.callback_enable, 1)
        self.sub_ball    = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        self.sub_joints  = self.create_subscription(JointState, "/joint_states", self.callback_joint_states, 1)
        self.pub_pantilt = self.create_publisher(Float32MultiArray, '/hardware/head/goal_pose', 1)


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
                        state = SM_LOOK_FOR_BALL
                    else:
                        None
                        
                elif state == SM_LOOK_FOR_BALL:
                    if not self.new_ball_data:
                        if self.get_clock().now() - self.last_image_time > rclpy.duration.Duration(seconds=2):
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
                    if self.new_ball_data:
                        self.last_image_time = self.get_clock().now()
                        self.new_ball_data = False
                        no_new_data_counter = 0
                        pantilt_msg = Float32MultiArray()
                        pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                        self.pub_pantilt.publish(pantilt_msg)
                    else:
                        no_new_data_counter += 1
                        if no_new_data_counter > 30:
                            self.get_logger().info("Lost ball!!!")
                            no_new_data_counter = 0
                            state = SM_LOOK_FOR_BALL
            #
            # END OF IF ENABLE
            #
            else:
                state = SM_INIT

            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    head_ball_follower_node = HeadBallFollowerNode()
    head_ball_follower_node.spin()
    head_ball_follower_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
