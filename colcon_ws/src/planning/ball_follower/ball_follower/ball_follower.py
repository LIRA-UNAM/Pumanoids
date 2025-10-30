import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pumas_vision_msgs.msg import VisionObject

center_x = 640
center_y = 360

class BallFollowerNode(Node):
    def callback_ball(self, msg):
        ball_center_x = msg.x
        ball_center_y = msg.y
        print(f"Ball detected at x: {ball_center_x}, y: {ball_center_y}")
        
        error_x = (ball_center_x - center_x)/640
        if error_x < 0:
            error_x = -math.sqrt(-error_x)
        
        else:
            error_x = math.sqrt(error_x)
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.2
        cmd_vel_msg.angular.z = -0.8 * error_x
        self.pub_cmd_vel.publish(cmd_vel_msg)
        
        
    def __init__(self):
        print("INITIALIZING BALL FOLLOWER NODE - ")
        super().__init__("ball_follower")
        self.sub_ball = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)
    

def main(args=None):
    rclpy.init(args=args)
    ball_follower= BallFollowerNode()
    rclpy.spin(ball_follower)
    ball_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
