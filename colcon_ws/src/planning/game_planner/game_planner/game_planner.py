#!/usr/bin/env python3

import rclpy
import socket
from rclpy.node import Node
from game_planner import gamestate
from construct import Enum
from std_msgs.msg import Bool

SOURCE_PORT = 3838
DESTINATION_PORT = 3939
COACH_PORT = 3839

class State(Enum):
    WAITING_CONNECTION = 0
    START = 1 
    POSITION_START = 2
    IDLE = 3
    ERROR = 4

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.get_logger().info('PlannerNode iniciado. Esperando Ctrl+C para cerrar.')
        #Publicadores
        self.position_start_enable_publisher = self.create_publisher(Bool, '/position/start/enable', 10)
        #Suscriptores 
        self.subscriber_ = self.create_subscription(Bool, '/position_start_finish', self.position_start_callback, 10)
        #Parametros 
        self.declare_parameter('host', "0.0.0.0")
        self.host = self.get_parameter('host').value
        self.get_logger().info(f'IP del host en {self.host}')
        #Variables
        self.position_start = False
        self.game_controller = None
        self.timer = self.create_timer(0.1, self.rustic_smach)
        self.state = State.START
        #Socket

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server_socket.bind((self.host,SOURCE_PORT))

        self.server_socket.settimeout(0.5)

    def position_start_callback(self, msg):
        self.position_start = msg.data
    
    def rustic_smach(self):

        try:
            data, addr = self.server_socket.recvfrom(1024)
            try:
                self.game_controller = gamestate.GameState.parse(data)
            except Exception as e:
                print(f"Un error ha ocurrido {e}")
                self.state = State.ERROR
        except socket.timeout:
            self.state = State.WAITING_CONNECTION


        if self.state == State.WAITING_CONNECTION:
            self.wait_state()
        elif self.state == State.START:
            self.start_state()
        elif self.state == State.POSITION_START:
            self.position_start_state()
        elif self.state == State.IDLE:
            self.idle_state()
        elif self.state == State.ERROR:
            self.error_state()
    
    def idle_state(self):
        self.get_logger().info("IDLE_STATE new states comming soon")
        self.state = State.IDLE

    def wait_state(self):
        self.get_logger().info("WAITING_CONNECTION waiting for game controller conection")
        self.state = State.START

    def start_state(self):
        self.get_logger().info("START_STATE waiting for ready from game controller")
        if self.game_controller.game_state == "STATE_READY":
            self.position_start_enable_publisher.publish(Bool(data = True))
            self.state = State.POSITION_START
        else:
            self.state = State.START

    def position_start_state(self):
        self.get_logger().info("POSITION_START_STATE walking to my position")
        if self.position_start:
            self.state = State.IDLE
        else:
            self.state = State.POSITION_START

    def error_state(self):
        self.get_logger().info("ERROR you shouldn't be here :c")
        self.state = State.IDLE


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
