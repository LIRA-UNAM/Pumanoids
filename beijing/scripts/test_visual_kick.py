#!/usr/bin/env python3
"""
Minimal test that publishes VisionKick (api_id=2038) commands directly to /LocoApiTopicReq.
Usage on the robot (after sourcing /opt/ros/humble/setup.bash):
  python3 test_visual_kick.py

The robot must already be upright in walking mode.
"""
import rclpy
from rclpy.node import Node
from booster_msgs.msg import RpcReqMsg
import json, uuid, time, sys

class VisionKickTest(Node):
    def __init__(self):
        super().__init__('vision_kick_test')
        self.pub = self.create_publisher(RpcReqMsg, 'LocoApiTopicReq', 10)
        time.sleep(1)  # Wait for the publisher connection.

    def send(self, api_id, body_dict=None):
        msg = RpcReqMsg()
        msg.uuid = str(uuid.uuid4())
        msg.header = json.dumps({"api_id": api_id})
        msg.body = json.dumps(body_dict) if body_dict else ""
        self.pub.publish(msg)
        self.get_logger().info(f"Sent api_id={api_id}, body={msg.body}")

    def visual_kick_start(self):
        self.send(2038, {"start": True})

    def visual_kick_stop(self):
        self.send(2038, {"start": False})

    def change_mode_walking(self):
        self.send(2000, {"mode": "walking"})

def main():
    rclpy.init()
    node = VisionKickTest()

    print("\n=== Minimal VisionKick Test ===")
    print("Ensure the robot is upright in walking mode")
    print("Place the ball 0.5-1.0 m in front of the robot")
    print()
    print("Commands:")
    print("  1 - Send VisionKick start (api_id=2038)")
    print("  2 - Send VisionKick stop  (api_id=2038)")
    print("  3 - Send ChangeMode walking (api_id=2000)")
    print("  q - Quit")
    print()

    try:
        while True:
            cmd = input("> ").strip()
            if cmd == '1':
                node.visual_kick_start()
            elif cmd == '2':
                node.visual_kick_stop()
            elif cmd == '3':
                node.change_mode_walking()
            elif cmd == 'q':
                break
            else:
                print("Invalid command")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
