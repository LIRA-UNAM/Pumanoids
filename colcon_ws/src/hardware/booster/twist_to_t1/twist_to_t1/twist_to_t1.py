import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from booster_interface.srv import RpcService

class TwistToT1Node(Node):
    def get_rpc_request(self):
        req = RpcService.Request()
        req.msg.api_id = 2001
        req.msg.body = "{\"vx\": " + str(self.vx) + ", \"vy\": " + str(self.vy) + ", \"vyaw\": " + str(self.vyaw) + "}"
        return req
    
    def callback_cmd_vel(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vyaw = msg.angular.z
        print(msg)
        return 
    
    def __init__(self):
        super().__init__('twist_to_t1')
        print("INITIALIZING TWIST TO T1 NODE - ")
        self.vx = 0.0
        self.vy = 0.0
        self.vyaw = 0.0
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback_cmd_vel, 10)
        self.clt_rpc = self.create_client(RpcService, '/booster_rpc_service')

    def spin(self):
        rpc_service_ready = False
        print("Waiting for rpc service...")
        while not self.clt_rpc.wait_for_service(timeout_sec=1.0):
            print('Waiting for rpc service...')
        print("rpc service is now available...")
        while rclpy.ok():
            if abs(self.vx) >= 0.0 or abs(self.vy) >= 0.0 or abs(self.vyaw) >= 0.0:
                req = self.get_rpc_request()
                future = self.clt_rpc.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                if future.result() is not None:
                    res = future.result()
                    # print('Response: %s' % res.msg.body)
                else:
                    self.get_logger().error('Service call failed %r' % (future.exception(),))
            rclpy.spin_once(self)
            time.sleep(0.5)



def main(args=None):
    rclpy.init(args=args)
    n = TwistToT1Node()
    n.spin()
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()