import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from booster_interface.srv import RpcService

class PantiltToT1Node(Node):
    def get_rpc_request(self):
        req = RpcService.Request()
        req.msg.api_id = 2004
        req.msg.body = "{\"pitch\": " + f"{self.pitch}" + ", \"yaw\": " + f"{self.yaw}" + "}"
        return req
    
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
        self.sub = self.create_subscription(Float32MultiArray, '/hardware/head/goal_pose', self.callback_pantilt_cmd, 10)
        self.clt_rpc = self.create_client(RpcService, '/booster_rpc_service')
        self.new_cmd = False
        
    def spin(self):
        rpc_service_ready = False
        print("Waiting for rpc service...")
        while not self.clt_rpc.wait_for_service(timeout_sec=1.0):
            print('Waiting for rpc service...')
        print("rpc service is now available...")
        while rclpy.ok():
            if self.new_cmd:
                self.new_cmd = False
                req = self.get_rpc_request()
                future = self.clt_rpc.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0) 
                if future.result() is not None:
                    res = future.result()
                else:
                    self.get_logger().error('Service call failed %r' % (future.exception(),))
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