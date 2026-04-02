#!/usr/bin/env python3

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
SM_LOOK_FOR_FACE = 20
SM_LOOK_AT_FACE = 30

class DeepFaceFollowerNode(Node):
    def callback_joint_states(self, msg):
        self.current_pan = msg.position[0]
        self.current_tilt = msg.position[1]
            
    def callback_face(self, msg):
        self.img_face_x = msg.x
        self.img_face_y = msg.y
        error_x = -(msg.x - self.img_goal_x) / (self.img_width  / 2)
        error_y =  (msg.y - self.img_goal_y) / (self.img_height / 2)        
        self.goal_pan += 0.15 * error_x
        self.goal_tilt += 0.15 * error_y
        self.goal_pan  = max(-1.0, min(1.0, self.goal_pan))
        self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))
        self.new_face_data = True

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
        super().__init__("deepface_follower_node")
        self.get_logger().info("INITIALIZING HEAD FACE FOLLOWER NODE - ")
        self.enable = True
        self.goal_pan = 0.0
        self.goal_tilt = 0.0
        self.new_face_data = False
        self.img_width  = 640
        self.img_height = 480
        self.img_goal_x = 320
        self.img_goal_y = 240
        self.img_face_x = 320
        self.img_face_y = 240
        self.current_pan  = 0
        self.current_tilt = 0
        self.look_for_poses = [[0.0, 0.0], [-0.8, 0.0], [-0.8, -0.2], [0.0, -0.2], [0.8, -0.2], [0.8, 0.0]]

        self.last_image_time = rclpy.time.Time(nanoseconds=0, clock_type=self.get_clock().clock_type)
        self.sub_face    = self.create_subscription(VisionObject, '/vision/face', self.callback_face, 1)
        self.sub_joints  = self.create_subscription(JointState, "/joint_states", self.callback_joint_states, 1)
        self.pub_pantilt = self.create_publisher(Float32MultiArray, '/hardware/head/goal_pose', 1)


    def spin(self):
        self.get_logger().info("Starting head follower loop...")
        state = SM_INIT
        no_new_data_counter = 0
        while rclpy.ok():
            if self.enable:
                if state == SM_INIT:
                    self.get_logger().info("Initializing state machine for head face follower...")
                    state = SM_WAIT_FOR_FIRST_IMAGE
                    
                elif state == SM_WAIT_FOR_FIRST_IMAGE:
                    img = self.get_single_image()
                    if img is not None:
                        self.get_logger().info(f"Image received with size {img.width}x{img.height}")
                        self.img_width  = img.width
                        self.img_height = img.height
                        self.img_goal_x = img.width/2
                        self.img_goal_y = img.height/2
                        state = SM_LOOK_FOR_FACE
                    else:
                        None
                        
                elif state == SM_LOOK_FOR_FACE:
                    if not self.new_face_data:
                        if self.get_clock().now() - self.last_image_time > rclpy.duration.Duration(seconds=2):
                            head_pose = self.look_for_poses.pop(0)
                            self.look_for_poses.append(head_pose)
                            self.get_logger().info(f"Looking for face at ({head_pose[0]},{head_pose[1]})")
                            pantilt_msg = Float32MultiArray()
                            pantilt_msg.data = head_pose
                            self.pub_pantilt.publish(pantilt_msg)
                            time.sleep(0.5)
                            rclpy.spin_once(self, timeout_sec=0.5)
                    else:
                        self.get_logger().info(f"Found face at position ({self.img_face_x},{self.img_face_y}) with head at ({self.current_pan},{self.current_tilt})")
                        self.goal_pan  = -0.65*(self.img_face_x - self.img_goal_x) / (self.img_width  / 2) + self.current_pan
                        self.goal_tilt =  0.50*(self.img_face_y - self.img_goal_y) / (self.img_height / 2) + self.current_tilt
                        self.goal_pan  = max(-1.0, min(1.0, self.goal_pan))
                        self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))
                        pantilt_msg = Float32MultiArray()
                        pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                        self.pub_pantilt.publish(pantilt_msg)
                        state = SM_LOOK_AT_FACE
                        
                elif state == SM_LOOK_AT_FACE:
                    if self.new_face_data:
                        self.last_image_time = self.get_clock().now()
                        self.new_face_data = False
                        no_new_data_counter = 0
                        pantilt_msg = Float32MultiArray()
                        pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                        self.pub_pantilt.publish(pantilt_msg)
                    else:
                        no_new_data_counter += 1
                        if no_new_data_counter > 30:
                            self.get_logger().info("Lost face!!!")
                            no_new_data_counter = 0
                            state = SM_LOOK_FOR_FACE
            else:
                state = SM_INIT

            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.02)

def main(args=None):
    rclpy.init(args=args)
    node = DeepFaceFollowerNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()