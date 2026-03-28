import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from pumas_vision_msgs.msg import VisionObject

center_x_t1 = 320
center_x_k1 = 272

class BallFollowerNode(Node):
    def callback_joint_states(self, msg):
        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]
        
    def callback_ball(self, msg):
        if not self.is_enabled:
            return

        ball_center_x = msg.x
        ball_center_y = msg.y
        
        error_x = (-ball_center_x + center_x_k1)/center_x_k1
        if error_x < 0:
            error_x = -math.sqrt(-error_x)
        else:
            error_x = math.sqrt(error_x)
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2
        cmd_vel_msg.angular.z = 0.8 * (error_x + self.current_head_pan)
        #self.get_logger().info(f"{error_x}  ,  {self.current_head_pan}")
        #self.get_logger().info(f"Pan position: {cmd_vel_msg.angular.z}")
        self.pub_cmd_vel.publish(cmd_vel_msg)
        
        
    def __init__(self):
        print("INITIALIZING BALL FOLLOWER NODE - ")
        super().__init__("ball_follower")
        self.is_enabled = False
        self.current_head_pan  = 0
        self.current_head_tilt = 0
        self.sub_ball = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
        self.sub_joints  = self.create_subscription(JointState, "/joint_states", self.callback_joint_states, 1)
        self.sub_enable = self.create_subscription(Bool, "/ball_follower/enable", self.callback_enable, 1)
        
        self.get_logger().info("Waiting for enable signal")
    
    def callback_enable(self, msg):
        if msg.data:
            if not self.is_enabled:
                self.get_logger().info("Ball Follower Enabled")
            self.is_enabled = True
        else:
            if self.is_enabled:
                self.get_logger().info("Ball Follower Disabled")
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(cmd_vel_msg)
            self.is_enabled = False
        

def main(args=None):
    rclpy.init(args=args)
    ball_follower= BallFollowerNode()
    rclpy.spin(ball_follower)
    ball_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
