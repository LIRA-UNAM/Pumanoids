import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool

class KickoffNode(Node):
    def __init__(self):
        super().__init__('kickoff_listener')
        self.subscription = self.create_subscription(
            Bool,
            'kickoff/enable',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Bool, 'ball_follower/enable', 10)

    def start_kick(self):
        print("Starting kicking sequence")
        self.publisher.publish(Bool(data=True))

    def wait_kick(self):
        print("Waiting for kickoff timeout")

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data:
            self.start_kick()
        else:
            self.wait_kick()


def main(args=None):
    rclpy.init(args=args)

    kickoffnode = KickoffNode()

    rclpy.spin(kickoffnode)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kickoffnode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
