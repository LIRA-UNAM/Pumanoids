#!/usr/bin/env python3
import time
import uuid
import json
import rclpy
from rclpy.node import Node
from brain.msg import Kick
from booster_msgs.msg import RpcReqMsg
from kick_interfaces.srv import KickBall

API_VISUAL_KICK = 2038

class K1KickNode(Node):
    def __init__(self):
        super().__init__("k1_kick_node")

        self.declare_parameter("publish_times", 80)
        self.declare_parameter("publish_rate_hz", 20.0)

        self.rpc_pub = self.create_publisher(RpcReqMsg, "/LocoApiTopicReq",10)

        self.kick_pub = self.create_publisher(Kick,"/kick_ball",1)

        self.kick_srv = self.create_service(KickBall, "/k1_kick", self.kick_callback)

        self.get_logger().info("Servicio listo: /k1_kick")

    
    def publish_rpc(self, api_id: int, body: dict):

        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())

        msg.header = json.dumps({"api_id": int(api_id), "expect_response": True})

        msg.body = json.dumps(body)
        self.rpc_pub.publish(msg)

    def visual_kick_start(self):
        self.publish_rpc(API_VISUAL_KICK, {"start": True, "version": 0})
    
    def visual_kick_stop(self):
        self.publish_rpc(API_VISUAL_KICK, {"start": False, "version": 0})

    def publish_kick_reference(self, request):
        publish_times = int(self.get_parameter("publish_times").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        if publish_rate_hz <= 0.0:
            publish_rate_hz = 20.0

        publish_times = max(1, publish_times)

        dt = 1.0 / publish_rate_hz

        msg = Kick()
        msg.header.frame_id = ""

        msg.x = float(request.ball_x)
        msg.y = float(request.ball_y)
        msg.dir = float(request.kick_dir)
        msg.goal_x = float(request.goal_x)
        msg.goal_y = float(request.goal_y)
        msg.robot_theta_to_field = float(request.robot_theta_to_field)
        msg.power = float(request.power)

        for i in range(publish_times):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.kick_pub.publish(msg)
            time.sleep(dt)

    def kick_callback(self, request, response):

        if request.ball_x <= 0.0:
            response.success = False
            response.message = "ball_x debe ser mayor que 0"
            return response

        if request.power <= 0.0:
            response.success = False
            response.message = "power debe ser mayor que 0"
            return response

        try:
            self.get_logger().info("Activando VisualKick...")
            self.visual_kick_start()

            # ESpera para asegurar haber entrado al modo VisualKick.
            time.sleep(0.1)

            self.publish_kick_reference(request)

            # Espera para asegurar no apagar VisualKick al tiempo de la ultima publicación 
            time.sleep(0.1)

            self.get_logger().info("Desactivando VisualKick...")
            self.visual_kick_stop()

            response.success = True
            response.message = "Secuencia de patada K1 publicada correctamente"
            return response


        except Exception as e:
            self.get_logger().error(f"Error durante la patada: {e}")

            try:
                self.visual_kick_stop()
            except Exception:
                pass

            response.success = False
            response.message = f"Error durante la patada: {e}"
            return response

def main(args=None):
    rclpy.init(args=args)

    node = K1KickNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
