import rclpy
import time
import json
import uuid
from rclpy.node import Node
from geometry_msgs.msg import Twist
from booster_msgs.msg import RpcReqMsg

class TwistToT1Node(Node):
    
    def __init__(self):
        super().__init__('twist_to_t1')
        print("INITIALIZING TWIST TO T1 NODE - ")
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.new_cmd_data = False
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)
        self.publisher = self.create_publisher(RpcReqMsg, '/LocoApiTopicReq', 10)

    def callback_cmd_vel(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vyaw = msg.angular.z
        self.new_cmd_data = True
        return 

    def build_move_msg(self, vx, vy, vyaw):
        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())
        msg.header = json.dumps({
            "api_id": 2001,                 # LocoApiId::kMove
            "expect_response": True
        })
        msg.body = json.dumps({
            "vx": vx,
            "vy": vy,
            "vyaw": vyaw
        })
        return msg

    def spin(self):
        no_new_cmd_counter = 0;
        while rclpy.ok():
            if self.new_cmd_data:
                self.new_cmd_data = False
                no_new_cmd_counter = 0
                msg = self.build_move_msg(self.vx, self.vy, self.vyaw)
                self.publisher.publish(msg)
            else:
                no_new_cmd_counter += 1
                if no_new_cmd_counter > 5:
                    no_new_cmd_counter = 0
                    msg = self.build_move_msg(0.0, 0.0, 0.0)
                    self.publisher.publish(msg)
                    self.get_logger().debug("No cmd_vel received, stopping robot")

            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(0.2)

def main(args=None):
    rclpy.init(args=args)
    n = TwistToT1Node()
    n.spin()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
