import rclpy
import time
import json
import uuid
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from booster_msgs.msg import RpcReqMsg

class PantiltToT1Node(Node):
    
    def callback_pantilt_cmd(self, msg):
        self.pitch = msg.data[1]
        self.yaw = msg.data[0]
        self.new_cmd = True
        # print(f"Received pantilt command - Pitch: {self.pitch}, Yaw: {self.yaw}")
        return
    
    def __init__(self):
        super().__init__('pantilt_to_t1')
        print("INITIALIZING PANTILT TO T1 NODE - ")

        self.pitch = 0.0
        self.yaw = 0.0
        self.new_cmd = False

        self.sub = self.create_subscription(Float32MultiArray, '/hardware/head/goal_pose', self.callback_pantilt_cmd, 10)
        
        self.publisher = self.create_publisher(RpcReqMsg, '/LocoApiTopicReq', 10)

    def build_head_msg(self, pitch, yaw):
        """Construct a RpcReqMsg for a RotateHead command."""
        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())
        msg.header = json.dumps({
            "api_id": 2004,                 # LocoApiId::kRotateHead
            "expect_response": True
        })
        msg.body = json.dumps({
            "pitch": pitch,
            "yaw": yaw
        })
        return msg

    def spin(self):
        while rclpy.ok():
            if self.new_cmd:
                self.new_cmd = False
                msg = self.build_head_msg(self.pitch, self.yaw)
                self.publisher.publish(msg)
                self.get_logger().debug(f"pitch: {self.pitch:.2f} , yaw: {self.yaw:.2f}")
            rclpy.spin_once(self)
            time.sleep(0.02)

def main(args=None):
    rclpy.init(args=args)
    n = PantiltToT1Node()
    n.spin()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
