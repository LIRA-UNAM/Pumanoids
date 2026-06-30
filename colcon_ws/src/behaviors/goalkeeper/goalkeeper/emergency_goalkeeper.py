import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class EmergencyGoalkeeper(Node):

    def __int__(self):
        super().__init__("emergency_goalkeeper")
        self.get_logger().info("INITIALIZING EMERGENCY GOALKEEPER")
        
        #--------PARAMETERS-------#
        # max dist of lateral movement
        # ros2 param set /emergency_goalkeeper amplitude_y 0.6
        self.declare_parameter('amplitude_y', 0.5)
        # time to complete one cicle
        self.declare_parameter('period',4.0)
        
        self.amplitude = self.get_parameter('amplitude_y').value
        self.period = self.get_parameter('period').value

        #---------VARIABLES--------#
        self.is_enabled = False
        self.start_time = None

        #---------TOPICS-----------#
        self.sub_enable = self.create_subscription(Bool,"/goal_keeper/enable", self.callback_enable,1)
        self.pub_cmd_vel = self.create_publisher(Twist,'/cmd_vel',1)

        #-------TIMERS---------#
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.main_timer)

        self.get_logger().info("Waiting for enable signal...")

    def callback_enable(self, msg):
        if msg.data:
            if not self.is_enabled:
                self.get_logger().info("Emergency goalkeeper Enabled")
                self.start_time = self.get_clock().now()
            self.is_enabled = True
        else:
            if self.is_enabled:
                self.get_logger().info("Emergency Goalkeeper Disabled")
                self.stop_robot()
            self.is_enabled = False

    def stop_robot(self):
        cmd_vel_msg = Twist()
        self.pub_cmd_vel.publish(cmd_vel_msg)

    def main_timer(self):
        if not self.is_enabled or self.start_time is None:
            return
        
        current_time = self.get_clock().now()
        t = (curr_time - self.start_time).nanoseconds / 1e9 #sec

        # Y speed
        omega = (2 * math.pi) / self.period
        v_y = self.amplitude * omega * math.cos(omega * t)

        # pub msg
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.y = v_y

        self.pub_cmd_vel.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    goalkeeper = EmergencyGoalkeeper()
    rclpy.spin(goalkeeper)
    goalkeeper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
