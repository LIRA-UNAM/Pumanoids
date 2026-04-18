# odom_bridge.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from booster_interface.msg import Odometer
from nav_msgs.msg import Odometry
import math

class OdomBridge(Node):
    def __init__(self):
        super().__init__('bridge_odom')

        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Odometer,
            '/odometer_state',
            self.callback,
            publisher_qos
        )
        self.pub = self.create_publisher(Odometry, '/odom_converted', 10)
        self.get_logger().info("Odom bridge running...")

    def callback(self, msg):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        # Posición directa desde booster_interface/msg/Odometer
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = 0.0

        # Convertir theta (yaw) a cuaternión
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(msg.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(msg.theta / 2.0)

        self.pub.publish(odom)

def main():
    rclpy.init()
    rclpy.spin(OdomBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
