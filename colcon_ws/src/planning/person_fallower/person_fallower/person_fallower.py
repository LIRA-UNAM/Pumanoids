#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from pumas_vision_msgs.msg import VisionObject

center_x_t1 = 320
center_x_k1 = 272

class PersonFollowerBase(Node):
    def callback_joint_states(self, msg):
        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]
            
    def callback_face(self, msg):
        if not self.is_enabled:
            return

        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2 #0.2
        
        # Control en Cascada: El cuerpo simplemente intenta llevar el cuello a 0.0
        cmd_vel_msg.angular.z = 1.0 * self.current_head_pan
        
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def __init__(self):
        print("INITIALIZING PERSON FOLLOWER NODE - ")
        super().__init__("person_fallower")
        self.is_enabled = False
        self.current_head_pan  = 0
        self.current_head_tilt = 0
        self.sub_face = self.create_subscription(VisionObject, '/vision/face', self.callback_face, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub_joints  = self.create_subscription(JointState, "/joint_states", self.callback_joint_states, 1)
        self.sub_enable = self.create_subscription(Bool, "/person_follower/enable", self.callback_enable, 1)
        
        self.get_logger().info("Waiting for enable signal")
    
    def callback_enable(self, msg):
        if msg.data:
            if not self.is_enabled:
                self.get_logger().info("Person Follower Enabled")
            self.is_enabled = True
        else:
            if self.is_enabled:
                self.get_logger().info("Person Follower Disabled")
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(cmd_vel_msg)
            self.is_enabled = False


def main(args=None):
    rclpy.init(args=args)
    person_follower= PersonFollowerBase()
    rclpy.spin(person_follower)
    person_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()