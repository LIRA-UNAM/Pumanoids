import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from booster_interface.srv import RpcService

class GoalkeeperGuard(Node):

def main(args=None):
    rclpy.init(args=args)
    n = TwistToT1Node()
    n.spin()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()