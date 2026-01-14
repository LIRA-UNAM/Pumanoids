import rclpy
import socket
#from gamestate import GameState
from rclpy.node import Node

from std_msgs.msg import Bool


from construct import Byte, Struct, Enum, Bytes, Const, Array, Int16ul, Int16ub, Int32ul, PaddedString, Flag, Int16sl

Short = Int16ul

RobotInfo = "robot_info" / Struct(
    "penalty" / Byte,
    "secs_till_unpenalized" / Byte,
    "number_of_warnings" / Byte,
    "number_of_yellow_cards" / Byte,
    "number_of_red_cards" / Byte,
    "goalkeeper" / Byte
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,
    "field_player_colour" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9
                        ),
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "coah_sequence" / Byte,
    "coach_message" / Bytes(253),
    "coach" / RobotInfo,
    "players" / Array(11, RobotInfo)
)

GameState = "gamedata" / Struct(
    "header" / Const(b'RGme'),
    "version" / Const(12, Short),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "game_type" / Byte,
    "game_state" / Enum(Byte,
                        STATE_INITIAL=0,
                        # auf startposition gehen
                        STATE_READY=1,
                        # bereithalten
                        STATE_SET=2,
                        # spielen
                        STATE_PLAYING=3,
                        # spiel zu ende
                        STATE_FINISHED=4
                        ),
    "first_half" / Flag,
    "kick_of_team" / Byte,
    "secondary_state" / Enum(Byte,
                             STATE_NORMAL=0,
                             STATE_PENALTYSHOOT=1,
                             STATE_OVERTIME=2,
                             STATE_TIMEOUT=3,
                             STATE_DIRECT_FREEKICK=4,
                             STATE_INDIRECT_FREEKICK=5,
                             STATE_PENALTYKICK=6,
                             STATE_CORNERKICK=7,
                             STATE_GOALKICK=8,
                             STATE_THROWIN=9,
                             DROPBALL=128,
                             UNKNOWN=255
                             ),
    "secondary_state_info" / Bytes(4),
    "drop_in_team" / Flag,
    "drop_in_time" / Short,
    "seconds_remaining" / Int16sl,
    "secondary_seconds_remaining" / Int16sl,
    "teams" / Array(2, "team" / TeamInfo)
)

GAME_CONTROLLER_RESPONSE_VERSION = 2

ReturnData = Struct(
    "header" / Const(b"RGrt"),
    "version" / Byte,
    "team" / Byte,
    "player" / Byte,
    "message" / Byte
)





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

    node = PlannerNode()

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
        

        if node.send:
            node.read(gamestate.game_state)

    Plannernode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
