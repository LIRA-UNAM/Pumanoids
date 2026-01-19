#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from enum import Enum
from std_msgs.std_msgs import Bool
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException
from pumas_vision_msgs.msg import VisionObject

def normalize_angle(a: float) -> float:
    return (a+math.pi)%(2.0*math.pi)-math.pi

def shortest_angular_distance(fr: float, to: float) -> float:
    return normalize_angle(to-fr)

def clamp(v: float, vmin: float, vmax: float) -> float:
    return max(vmin, min(vmax, v))

def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosṕ = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class State(Enum):
    Waiting_State = 0
    Center_Ball = 1
    Approach_ball = 2
    Orbit_ball = 3
    Finished = 4

class MoveToBall(Node):
    def __init__(self):
        super().__init__("move_to_ball")

        self.declare_parameter("timer_period_ms",50)
        self.declare_parameter("enable_topic","/move_to_ball/enable")
        self.declare_parameter("finish_topic","/move_to_ball/finish")
        self.declare_parameter("ball_topic","/vision/ball")
        self.declare_parameter("cmd_vel_topic","/cmd_vel")
        self.declare_parameter("ball_timeout_s",0.5)

        self.declare_parameter("target_distance_m",0.50)
        self.declare_parameter("distance_tol_m",0.05)

        self.declare_parameter("center_tol_y_m",0.05)
        self.declare_parameter("kp_center_y",1.2)

        self.declare_parameter("kp_forward",0.9)

        self.declare_parameter("max_lin_x",0.35)
        self.declare_parameter("max_lin_x_back",0.2)
        self.declare_parameter("max_lin_y",0.25)
        self.declare_parameter("max_ang_z",0.90)

        self.declare_parameter("hold_finish",True)

        self.cmd_pub_ = self.create_publisher(Twist, self.get_parameter("cmd_vel_topic").value,10)
        self.finish_pub_ = self.create_publisher(Bool, self.get_parameter("cmd_vel_topic").value,10)
        self.enable_sub_ = self.create_subscription(Bool, self.get_parameter("enable_topic").value, self.callback_enable, 10)
        self.ball_sub_ = self.create_suscription(VisionObject, self.get_parameter("ball_topic").value, self.callback,10)

        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_, self)

        period = int(self.get_parameter("timer_period_ms").value)/1000.0
        self.timer_ = self.create_timer(period, self.timer_callback)

        self.current_state_ = State.Waiting_State
        self.enabled_ = False
        self.finish_sent_ = False

        self.last_ball_time_ = None
        self.ball_x_m_ = None
        self.ball_y_m_ = None

        self.orbit_target_yaw_ = None
        self.get_logger().info("Move_to_ball node started")

        #Callback: enable/disable

    def callback_enable(slef, msg: Bool):
        if msg.data and not self.enabled_:
            self.enabled_ = True
            self.finish_sent_ = False
            self.orbit_target_yaw_ = None
            self.current_state_ = State.Center_Ball
            self.get_logger().info("ENABLE -> CENTER_BALL")
            
        if (not msg.data) and self.enabled_:
            self.enabled_ = False
            self.current_state_ = State.Waiting_State
            self.finish_sent_ = False
            self.orbit_target_yaw_ = None
            self._publish_zero_cmd()
            self.get_logger().info("Disable -> Waiting_State")

        
        #Callback: /vision/ball

    def callback_ball(self, msg:VisionObject):
        self.last_ball_time_ = self.get_clock().now()
        self.ball_x_m_ = float(msg.pose.position.x)
        self.ball_y_m = float(msg.pose.position.y)
        
        # Timer callback for state machine

    def timer_callback(self):
        if not self.enabled_:
            return
            
        if self.current_state_ == State.Finished:
            self._publish_zero_cmd()
            if not self.finish_sent_:
                m = Bool()
                m.data = True
                self.finish_pub_.publish(m)
                self.finish_sent_ = True
                self.get_logger().info("Move_to_ball finished")
            
        if not bool(self.get_parameter("holf_finished").value):
            self.current_state_ = State.Waiting_State
            self.enabled_ = False
        return

        if not self._ball_is_fresh():
            self._publish_zero_cmd()
            if self.current_state_ != State.Center_Ball:
                self.current_state_ = State.Center_Ball
                self.get_logger().warm("Ball timeout -> Center_Ball")
            return

        yaw = self._get_base_yaw()
        if yaw is None:
            self._publish_zero_cmd()
            return

        bx = float(self.ball_x_m_)
        by = float(self.ball_y_m_)
        r = math.hypot(bx, by)
        bearing = math.atan2(by,bx)

        target_dist = float(self.get_parameter("target_distance_m").value)
        dist_tol = float(self.get_parameter("distance_tol_m").value)
        center_tol_y = float(self.get_parameter("center_tol_y_m").value)
        kp_center_y = float(self.get_parameter("kp_center_y").value)
        kp_forward = float(self.get_parameter("kp_forward").value)

        orbit_angle = math.radians(float(self.get_parameter("orbit_angle_deg").value))
        orbit_dir = int(self.get_parameter("orbit_direction").value)
        orbit_lin_y = float(self.get_parameter("orbit_lin_y").value)
        kp_orbit_radius = float(self.get_parameter("kp_orbit_radius").value)
        kp_face_ball = float(self.get_parameter("kp_face_ball").value)
        yaw_tol = float(self.get_parameter("orbit_yaw_tol_rad").value)

        max_lin_x = float(self.get_parameter("max_lin_x").value)
        max_lin_x_back = float(self.get_parameter("mas_lin_x_back").value)
        max_lin_y = float(self.get_parameter("max_lin_y").value)
        max_ang_z = float(self.get_parametes("max_ang_z").value)

        twist = Twist()

        if self.current_state_ == State.Center_Ball:
            if abs(by) > center_tol_y:
                twist.linear.y = clamp(kp_center_y*by, -max_lin_y, max_lin_y)
                self.cmd_pub_.publish(twist)
            else:
                self._publish_zero_cmd()
                self.current_state_ = State.Approach_ball
                self.get_logger().info("Centered -> Approach_ball")
            return
            
        if self.current_state_ == State.Approach_ball:
            dist_err = r - target_dist

            if dist_err > dist_tol:
                vx = clamp(kp_forward * dist_err, 0.05, max_lin_x)
                vy = clamp(kp_center_y * by, - max_lin_y, max_lin_y)
                twist.linear.x = vx
                twist.linear.y = vy
                self.cmd_pub_.publish(twist)
                
            else:
                self._publish_zero_cmd()
                self.orbit_target_yaw_ = normalize_angle(yaw + orbit_dir + orbit_angle)
                self.current_state_ = State.Orbit_ball
                self.get_logger().info(f"Reached {r:.2f}m -> Orbit_ball target_yaw={self.orbit_target_yaw_:.3f}")
                
            return
            
        if self.current_state_ == State.Orbit_ball:
            if self.orbit_target_yaw_ is None:
                self.orbit_target_yaw_ = normalize_angle(yaw + orbit_dir * orbit_angle)
                
            yaw_err = shortest_angular_distance(yaw, self.orbit_target_yaw_)
                
            if abs(yaw_err) > yaw_tol:
                twist.linear.y = clamp(orbit_dir * orbit_lin_y, -max_lin_y, max_lin_y)
                rad_err = r - target_dist
                vx = clamp(kp_orbit_radius*rad_err, -max_lin_x_back,max_lin_x)
                twist.linear.x = vx

                twist.angular.z = clamp(kp_face_ball * bearing, -max_ang_z, max_ang_z)

                self.cmd_pub_.publish(twist)
            else:
                self._publish_zero_cmd()
                self.current_state_ = State.Finished
                self.get_logger().info("Orbit complete -> FINISHED")
            return
        
    def _publish_zero_cmd(self):
        self.cmd_pub_.publish(Twist())

    def _ball_is_fresh(self) -> bool:
        if self.last_ball_time_ is None or self.ball_x_m_ is None or self.ball_y_m_ is None:
            return False
        timeout_s = float(self.get_parameter("ball_timeout_s").value)
        age = (self.get_clock().now() - self.last_ball_time_).nanoseconds * 1e-9
        return age <= timeout_s
        
    def _get_base_yaw(self):
        try:
            t = self.tf_buffer_.lookup_transform("odom", "base_link", rclpy.time.Time())
            return normalize_angle(yaw_from_quaternion(t.transform.rotation))
        except TransformException as ex:
            self.get_logger().error(f"TF odom -> base_link failed: {ex}")
            return None
        
def main(args=None):
    rclpy.init(args=args)
    node = MoveToBall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
