import rclpy
from gamestate import GameState
from rclpy.node import Node

from std_msgs.msg import Bool

class PlannerNode(Node):
    def __init__(self):
        super().__init__('Planner_listener')
        self.subscription = self.create_subscription(
            Bool,
            '/position_start/finish',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Bool, '/position_start/enable', 10)
        self.send = True

    def read(self, state):
        if state == "STATE_READY":
            self.publisher.publish(Bool(data = True))

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data:
            self.publisher.publish(Bool(data = False))
            self.send = False


def main(args=None):
    host = "0.0.0.0"
    port = 3838
    print('Initializing')
    rclpy.init(args=args)

    Plannernode = PlannerNode()

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    server_socket.bind((host,port))

    server_socket.settimeout(0.5)

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)

        try:
            data, addr = server_socket.recvfrom(1024)
        except socket.timeout:
            continue

        try:
            gamestate = GameState.parse(data)
        except Exception as e:
            node.get_logger().warn(f"Un error ha ocurrido {e}")
            continue
        

        if PlannerNode.send:
            PlannerNode.read(gamestate.game_state)

    Plannernode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
