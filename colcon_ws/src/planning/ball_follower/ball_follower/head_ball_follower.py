import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.wait_for_message import wait_for_message
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, JointState
from pumas_vision_msgs.msg import VisionObject
from joint_states_package.srv import HeadJoints

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
            self.get_logger().error(f'Joint service call: {e}')


            
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
            topic=self.get_parameter('camera_topic').get_parameter_value().string_value,
            qos_profile=qos_profile,
            time_to_wait=timeout_seconds
        )
        return msg if success else None
        
    def __init__(self):
        super().__init__("head_ball_follower")
        self.get_logger().info("INITIALIZING HEAD BALL FOLLOWER NODE - ")

        # --- VARIABLES ---
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
        self.look_for_poses = [[0.0,0.7], [-0.8, 0.7], [-0.8, 0.2], [0.0, 0.2], [0.8, 0.2], [0.8,0.7]]
        self.search_pose_step_count = 0
        self.search_cycle_counter = 0
        self.timer_ang_vel = 0.7
        self.main_timer_period = 0.1
        self.rotation_timer_period = 0.1
        self.head_timer_period = 1
        self.timer_counter = 0
        self.timer_enable = False
        self.head_timer_enable = False
        self.last_head_pan  = 0     # Lastest received head pan (yaw) position
        self.last_head_tilt = 0     # Lastest received head tilt (pitch) position

        self.declare_parameter("camera_topic", "/boostercamera/head/rgb")
        self.declare_parameter("body_search_cycles_before_turn", 1)
        self.declare_parameter("body_search_turn_deg", 90.0)
        self.declare_parameter("body_search_ang_vel", 0.7)
        self.declare_parameter("body_search_turn_sign", 1.0)
        self.declare_parameter("enable_body_search_turn", True)

        # -- QoS PROFILES --
        qos_profile_for_enabling = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        self.last_image_time = rclpy.time.Time(nanoseconds=0, clock_type=self.get_clock().clock_type)

        # --- TOPICS ---
        self.sub_enable  = self.create_subscription(Bool, "/head_ball_follower/enable", self.callback_enable, qos_profile_for_enabling)
        self.sub_ball    = self.create_subscription(VisionObject, '/vision/ball', self.callback_ball, 1)
        self.pub_pantilt = self.create_publisher(Float32MultiArray, '/hardware/head/goal_pose', 1)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- SERVICES ---
        # Jont positions client
        self.cli = self.create_client(HeadJoints, 'get_head_joints')

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service get_head_joints not available')

        # --- TIMERS ---
        # For requesting joint states
        self.create_timer(0.05, self.joint_state_request)

        self.main_timer_ = self.create_timer(self.main_timer_period, self.main_timer_callback)
        self.rotation_timer_ = self.create_timer(self.rotation_timer_period, self.rotation_timer_callback)
        self.head_timer_ = self.create_timer(self.head_timer_period, self.head_rotation_timer_callback)

        self.get_logger().info("Waiting for enable signal")
        self.State_ = SM_INIT
        self.Prev_state_ = SM_INIT
        self.no_new_data_counter_ = 0
        self.last_enable_ = False

    def _publish_cmd_vel_zero(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        
        self.pub_cmd_vel.publish(twist)

    def head_rotation_timer_callback(self):
        if self.head_timer_enable:
            self.get_logger().debug("Rotation head ENABLED")
            # Assign a head position to head_pose and move it to the end of the list to create a cycle.
            head_pose = self.look_for_poses.pop(0)
            self.look_for_poses.append(head_pose)
            self.search_pose_step_count += 1
            nposes = len(self.look_for_poses)
            # Each time we complete a cycle through all the predefined head poses, increment the search cycle counter
            if nposes > 0 and self.search_pose_step_count % nposes == 0:
                self.search_cycle_counter += 1
            # If we complete a number of cycles (cycles_needed_before_turn), enable turning the robot
            pantilt_msg = Float32MultiArray()
            pantilt_msg.data = head_pose
            self.get_logger().debug(f"Looking for ball at ({head_pose[0]},{head_pose[1]})")
            self.pub_pantilt.publish(pantilt_msg)
        else:
            self.get_logger().debug("Rotation head DISABLED")

    # Main function that implements the state machine for the head ball follower, following a timer-based approach.
    def main_timer_callback(self):
        # We check if the logic is enabled by the game_planner node
        if self.enable:
            # --- INITIAL STATE ---
            if self.State_ == SM_INIT:
                self.get_logger().info("Initializing state machine for head ball follower...")
                self.State_ = SM_WAIT_FOR_FIRST_IMAGE
            
            # --- WAIT FOR FIRST IMAGE ---
            elif self.State_ == SM_WAIT_FOR_FIRST_IMAGE:
                img = self.get_single_image()
                if img is not None:
                    self.get_logger().info(f"Image received with size {img.width}x{img.height}")
                    self.img_width  = img.width
                    self.img_height = img.height
                    self.img_goal_x = img.width/2
                    self.img_goal_y = img.height/2
                    # After receiving image, we can start looking for the ball
                    self.State_ = SM_LOOK_FOR_BALL
                else:
                    None
            
            # --- LOOKING FOR BALL ---
            elif self.State_ == SM_LOOK_FOR_BALL:
                # If it's the first time we enter this state or we just lost the ball, reset the search counters
                if self.Prev_state_ != SM_LOOK_FOR_BALL:
                    self.search_pose_step_count = 0 # Head poses
                    self.search_cycle_counter = 0   # Cycles for head poses
                # If there's no ball, the robot looks for it
                if not self.new_ball_data:
                    self.head_timer_enable = True
                    cycles_needed_before_turn = int(self.get_parameter("body_search_cycles_before_turn").value)
                    if self.search_cycle_counter >= cycles_needed_before_turn:
                        self.head_timer_enable = False
                        self._execute_body_search_turn()
                        self.search_cycle_counter = 0
                # If there's a ball. Look at it and switch to LOOK AT BALL state
                else:
                    # STOP whatever mafofada the robot was doing. Then proceed
                    self._publish_cmd_vel_zero()
                    self.get_logger().info(f"Found ball at position ({self.img_ball_x},{self.img_ball_y}) with head at ({self.last_head_pan},{self.last_head_tilt})")
                    self.goal_pan  = -0.65*(self.img_ball_x - self.img_goal_x) / (self.img_width  / 2) + self.last_head_pan
                    self.goal_tilt =  0.50*(self.img_ball_y - self.img_goal_y) / (self.img_height / 2) + self.last_head_tilt
                    self.goal_pan  = max(-1.0, min(1.0, self.goal_pan))
                    self.goal_tilt = max(-0.3, min(0.8, self.goal_tilt))
                    pantilt_msg = Float32MultiArray()
                    pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                    self.pub_pantilt.publish(pantilt_msg)
                    self.State_ = SM_LOOK_AT_BALL
            
            # --- BALL EXIST. LOOK AT BALL ---
            elif self.State_ == SM_LOOK_AT_BALL:
                if self.new_ball_data:
                    # First, make sure other variables are reset.
                    self.search_cycle_counter = 0
                    self.timer_enable = False
                    self.head_timer_enable = False
                    self.last_image_time = self.get_clock().now()
                    self.no_new_data_counter_ = 0
                    # Set the ball data flag to false.
                    self.new_ball_data = False
                    # Look at the ball
                    pantilt_msg = Float32MultiArray()
                    pantilt_msg.data = [self.goal_pan, self.goal_tilt]
                    self.pub_pantilt.publish(pantilt_msg)
                # Lost the ball
                else:
                    # Add a counter for tolerance
                    self.no_new_data_counter_ += 1
                    if self.no_new_data_counter_ > 30: # <-- adjust as needed
                        self.get_logger().info("Lost ball!!!")
                        self.no_new_data_counter_ = 0
                        self.State_ = SM_LOOK_FOR_BALL

                self.Prev_state_ = self.State_
            else:
                self.get_logger().error(f"Unknown state: {self.State_}")
            
            # DISABLED RECEIVED
        else:
            # If the enable signal is disabled, we reset the state machine and stop the robot if it was enabled before.
            if self.last_enable_:
                self._publish_cmd_vel_zero()
            # Reset everything
            self.State_ = SM_INIT
            self.Prev_state_ = SM_INIT
            self.no_new_data_counter_ = 0
            self.last_enable_ = False
            self.search_pose_step_count = 0
            self.search_cycle_counter = 0
            self.timer_enable = False
            self.head_timer_enable = False

        self.last_enable_ = self.enable
            


    def rotation_timer_callback(self):
        if self.timer_enable:
            self.timer_counter += 1
            if self.timer_counter >= 15:
                self._publish_cmd_vel_zero()
                self.timer_enable = False
                self.timer_counter = 0
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = self.timer_ang_vel
            self.pub_cmd_vel.publish(twist)
        else:
            self.timer_counter = 0

    
    def _execute_body_search_turn(self):
        if not self.get_parameter("enable_body_search_turn").value:
            return
        self.timer_enable = True


def main(args=None):
    rclpy.init(args=args)
    head_ball_follower_node = HeadBallFollowerNode()
    rclpy.spin(head_ball_follower_node)
    head_ball_follower_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
