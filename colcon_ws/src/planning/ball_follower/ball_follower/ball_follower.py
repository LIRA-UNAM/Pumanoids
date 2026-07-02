import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.wait_for_message import wait_for_message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from pumas_vision_msgs.msg import VisionObject
from joint_states_package.srv import HeadJoints

# Image center (in pixels) for every robot model
center_x_t1 = 640
center_x_k1 = 360

class BallFollowerNode(Node):

    def __init__(self):
        print("INITIALIZING BALL FOLLOWER NODE - ")
        super().__init__("ball_follower")

        # --- PARAMETERS ---
        self.declare_parameter("camera_topic", "/boostercamera/head/rgb")

        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('linear_speed', 0.3)

        # --- VARIABLES ---
        self.is_enabled = False     # To check enable status from game_planner
        self.last_head_pan  = 0     # Lastest received head pan (yaw) position
        self.last_head_tilt = 0     # Lastest received head tilt (pitch) position

        self.ball_detected = False  # To check if a ball has been detected by ball_detector
        self.ball_center_x = 0    # Ball position in the camera in the X axis
        self.ball_center_y = 0    # Ball position in the camera in the Y axis

        self.first_image = False # To check if we got the first image

        self.img_width  = None
        self.img_height = None
        self.img_goal_x = None
        self.img_goal_y = None


        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # --- TOPICS ---
        # Enabling signal from game_planner node
        self.sub_enable = self.create_subscription(Bool, "/ball_follower/enable", self.callback_enable, 1)

        # To receive detected objects from ball_detector node
        self.sub_ball = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        
        # To send high-level movement commands to twist_to_x1 node (replacing x1 with your robot model)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 1)

        # --- SERVICES ---
        # Jont positions client
        self.cli = self.create_client(HeadJoints, 'get_head_joints')

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service get_head_joints not available')

        # --- TIMERS ---
        # For requesting joint states
        self.create_timer(0.1, self.joint_state_request)
        # The main timer callback function. Most of the node logic goes here.
        self.create_timer(0.1, self.main_timer) # <-- Needs testing to adjust timer period. Starting at 0.1


        self.get_logger().info("Waiting for enable signal")

    # Callback function of the enable signal from the game_planner
    def callback_enable(self, msg):
        # TRUE received
        if msg.data:
            if not self.is_enabled:
                self.get_logger().info("Ball Follower Enabled")
            self.is_enabled = True
        # FALSE received
        else:
            # If it was enabled, we stop everything
            if self.is_enabled:
                self.get_logger().info("Ball Follower Disabled")
                cmd_vel_msg = Twist()
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.pub_cmd_vel.publish(cmd_vel_msg)
            self.is_enabled = False

    # Request function for joint states
    def joint_state_request(self):
        if not self.cli.service_is_ready():
            return

        req = HeadJoints.Request()

        future = self.cli.call_async(req)
        future.add_done_callback(self.joints_response_callback)

    # Response function for joint states
    def joints_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.last_head_pan = response.pan
                self.last_head_tilt = response.tilt

                self.get_logger().debug(f'Head joints: pan={response.pan:.3f}, tilt={response.tilt:.3f}', throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f'Joint service callback: {e}')

    def get_single_image(self, timeout_seconds=1):
        self.get_logger().info("Waiting for single image")
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        success, msg = wait_for_message(
            msg_type=Image,
            node=self,
            topic=self.get_parameter('camera_topic').get_parameter_value().string_value,
            qos_profile=qos_profile,
            time_to_wait=timeout_seconds
        )
        return msg if success else None

    
    # When ball_detector finds the ball, we store it's position in pixel coordinates
    def callback_ball(self, msg):

        if not self.is_enabled:
            return

        self.ball_detected = True
        self.ball_center_x = msg.x
        self.ball_center_y = msg.y
        
        
    def main_timer(self):

        if not self.first_image:
            img = self.get_single_image()
            if img is not None:
                self.get_logger().info(f"Image received with size {img.width}x{img.height}")
                self.img_width  = img.width
                self.img_height = img.height
                self.img_goal_x = img.width/2
                self.img_goal_y = img.height/2
                self.first_image = True
            else:
                return

        if not self.is_enabled or not self.ball_detected:
            return

        if self.img_goal_x is None:
            self.get_logger().info("NO img center")
            return

        # Return the ball_detected flag to false
        self.ball_detected = False
        # Calculate the horizontal error from center to ball position
        error_x = (self.img_goal_x - self.ball_center_x) / self.img_goal_x
        if error_x < 0:
            error_x = -math.sqrt(-error_x)
        else:
            error_x = math.sqrt(error_x)
        
        # Send the movement command. The robot rotates depending on the horizontal position of the ball, taking in consideration the position of the ball in the camera and the head (yaw) position.
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = self.linear_speed 
        cmd_vel_msg.angular.z = self.angular_speed * (error_x + self.last_head_pan)
        #self.get_logger().info(f"{error_x}  ,  {self.current_head_pan}")
        #self.get_logger().info(f"Pan position: {cmd_vel_msg.angular.z}")
        self.pub_cmd_vel.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    ball_follower= BallFollowerNode()
    rclpy.spin(ball_follower)
    ball_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
