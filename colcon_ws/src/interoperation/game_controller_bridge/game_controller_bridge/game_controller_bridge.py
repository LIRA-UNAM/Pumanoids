import socket
import rclpy
#from gamestate import GameState
#import game_controller_bridge.utils
#from game_controller_msgs.msgs import GameStateInfo
from std_msgs.msg import String
from construct import Byte, Struct, Enum, Bytes, Const, Array, Int16ul, Int32ul, PaddedString, Flag, Int16sl
Short = Int16ul


RobotInfo = "robot_info" / Struct(
    # define NONE                        0
    #define HL_BALL_MANIPULATION                30
    #define HL_PHYSICAL_CONTACT                 31
    #define HL_ILLEGAL_ATTACK                   32
    #define HL_ILLEGAL_DEFENSE                  33
    #define HL_PICKUP_OR_INCAPABLE              34
    #define HL_SERVICE                          35
    "penalty" / Byte,
    "secs_till_unpenalized" / Byte,
    "number_of_warnings" / Byte,
    "number_of_yellow_cards" / Byte,
    "number_of_red_cards" / Byte,
    "goalkeeper" / Flag
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,
    "team_color" / Enum(Byte,
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
    "coach_sequence" / Byte,
    "coach_message" / PaddedString(253, 'utf8'),
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

def main(args=None):
    print('Initializing')
    rclpy.init(args=args)
    node = rclpy.create_node('GameStateNode')
    pub_game_state = node.create_publisher(String, '/game_control/state',10)
    node.declare_parameter('host','0.0.0.0')
    host = node.get_parameter('host').value
    port = 3838
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    server_socket.bind((host,port))

    server_socket.settimeout(0.5)

    try:
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

            gamestate_msg = String()

            gamestate_msg.data=str(gamestate.game_state)

            # gamestate_msg.gc_header = gamestate.header.decode("utf-8")
            # gamestate_msg.protocol_version = gamestate.version
            # gamestate_msg.packet_number = gamestate.packet_number
            # gamestate_msg.players_per_team = gamestate.players_per_team
            # gamestate_msg.game_type = gamestate.game_type
            # gamestate_msg.state = gamestate.game_state
            # gamestate_msg.first_half = gamestate.first_half
            # gamestate_msg.kick_off_team = gamestate.kick_of_team
            # gamestate_msg.secondary_state = gamestate.secondary_state
            # gamestate_msg.secondary_state_info = gamestate.secondary_state_info
            # gamestate_msg.drop_in_team = gamestate.drop_in_team
            # gamestate_msg.drop_in_time = gamestate.drop_in_time
            # gamestate_msg.secs_remaining = gamestate.seconds_remaining
            # gamestate_msg.secondary_time = gamestate.secondary_seconds_remaining

            pub_game_state.publish(gamestate_msg)

    except KeyboardInterrupt:
        pass
    finally:
        server_socket.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
