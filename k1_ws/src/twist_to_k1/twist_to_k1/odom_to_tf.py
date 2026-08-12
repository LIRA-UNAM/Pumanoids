import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from booster_interface.msg import Odometer


class OdomToTFNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.subscription = self.create_subscription(Odometer,'/odometer_state',self.callback_odometer,10)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_a = 0.0

        self.get_logger().info("Booster Odom to TF Node has been started.")

    def callback_odometer(self, msg: Odometer):
        self.robot_x = float(msg.x)
        self.robot_y = float(msg.y)
        yaw = float(msg.theta)
        self.robot_a = (yaw + math.pi) % (2.0 * math.pi) - math.pi

    def timer_callback(self):
        t = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = t
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.robot_x
        transform.transform.translation.y = self.robot_y
        transform.transform.translation.z = 0.0

        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.robot_a)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.broadcaster.sendTransform(transform)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return (qx, qy, qz, qw)


def main():
    rclpy.init()
    node = OdomToTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
