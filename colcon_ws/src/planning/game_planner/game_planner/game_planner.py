#!/usr/bin/env python3

# Node: game_planner
# The general state machine node to receive Game Controller messages and manage the robot behavior.
# Written by Camile Frias and Sebastian Garcia.
# Developed at LIRA UNAM.
# https://lira.unam.mx/

import rclpy
import socket
from rclpy.node import Node
from game_planner import gamestate
from construct import Enum
from std_msgs.msg import Bool

# Ports to comunicate with the Game Controller under UDP packages
SOURCE_PORT = 3838 # Game Controller broadcast messages through port 3838
DESTINATION_PORT = 3939 # This node sends messages to Game Controller through port 3939
COACH_PORT = 3839 # To communicate with the coach

# -- RETURN_MSGS --
PENALISE=                    0
UNPENALISE=                  1
ALIVE=                       2
GOALKEEPER=                  3
INTERRUPTION_READY=          4


# The states of the node 
class State(Enum):
    WAITING_CONNECTION = 0 # Not receiving any signal from the Game Controller
    START = 1 # Game controller STATE_READY. The initial state of Game Controller
    POSITION_START = 2 # The robots move from the side to their kickoff positions
    SET = 3 # Game controller STATE_SET. Robots must not move in this state.
    KICKOFF = 4 # Game controller STATE_PLAYING. Attack (follow_ball) or defense (wait to ball to move and then follow_ball).
    PLAYING = 5 # To follow the ball detected by the robot's camrea. If goalkeeper, run the defense node.
    FINISH = 6 # THE END
    IDLE = 7 # IDLE
    ERROR = 8 # Not good
    # More comming soon

class PlannerNode(Node):
    def __init__(self):
        super().__init__('game_planner')
        self.get_logger().info('game_planner iniciado. Esperando...') 
        self.get_logger().info('Ctrl+C para cerrar.')
        
        # -- PUBLISHERS --
        # position_start
        self.head_ball_follower_enable_publisher = self.create_publisher(Bool, '/head_ball_follower/enable', 10)
        self.ball_follower_enable_publisher = self.create_publisher(Bool, '/ball_follower/enable', 10)
        self.position_start_enable_publisher = self.create_publisher(Bool, '/position_start/enable', 10)
        
        # -- SUBSCRIBERS --
        # position_start
        self.position_start_enable_subscriber = self.create_subscription(Bool, '/position_start/finish', self.position_start_callback, 10)
        #TODO MOVINGBALL subscriber
        
        # -- PARAMETERS --
        self.host ="0.0.0.0" # Always watching any IP

        #player_number
        self.declare_parameter('player_number', 1)
        self.player_number = self.get_parameter('player_number').value
        self.get_logger().info(f'Jugador {self.player_number} Listo')

        #team_number
        self.declare_parameter('team_number', 0)
        self.team_number = self.get_parameter('team_number').value
        self.get_logger().info(f'Equipo número {self.team_number} PUMANOIDS')

        #Is goalkeeper?
        self.declare_parameter('goalkeeper', False)
        self.goalkeeper = self.get_parameter('goalkeeper').value
        self.get_logger().info(f'Portero? {self.goalkeeper} ')


        
        # VARIABLES
        self.position_start = False
        self.move_ball = False
        self.game_controller = None
        self.target_team = None
        self.connection_timeout = 0.7 # <-- Adjust this to set the connection tolerance in seconds :)
        self.last_packet_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.rustic_smach) # Timer for the rustic_smach function.
        self.current_state = State.WAITING_CONNECTION
        self.last_available_state = State.WAITING_CONNECTION
        
        # -- SOCKETS --
        #Broadcast from game_controller
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server_socket.bind((self.host,SOURCE_PORT))
        
        self.server_socket.setblocking(False)
    # -- CALLBACKS --    
    def position_start_callback(self, msg):
        self.position_start = msg.data
    
    #TODO do the move_ball callback after the subscriber

    # Primary function of the state machine.
    # This function is called by the timer declared above.
    def rustic_smach(self):

        try:
            # Loop to get the latest packet and clear the buffer
            # Once the buffer is empty, it will exit the while loop.
            while True:
                # Attempts to read up to 1024 bytes from the UDP socket.
                data, addr = self.server_socket.recvfrom(1024)
                self.get_logger().debug(f"Received data from {addr}")
                # Stores the data.
                self.game_controller = gamestate.GameState.parse(data)
                # Update last connection timestamp.
                self.last_packet_time = self.get_clock().now()

                #Game_controller addres
                gc_ip = addr[0]
                #Message with number and if is or not goalkeeper
                return_message = gamestate.ReturnData.build(dict(
                version=gamestate.GAME_CONTROLLER_RESPONSE_VERSION,
                team=self.team_number,
                player=self.player_number,
                message=GOALKEEPER if self.goalkeeper else ALIVE
                )) 
                #Send message
                self.server_socket.sendto(return_message, (gc_ip, DESTINATION_PORT))
                
        # If we clear the buffer, this exception gets caught. 
        except (BlockingIOError):
            # This is expected.
            pass

        # Catch any unexpected exception
        except Exception as e:
            self.get_logger().error(f"UDP socket error: {e}")
            # If you're getting a timeout error (not warning), chech that setblocking is set to False in the lines above.

        # Check for timeout
        if (self.get_clock().now() - self.last_packet_time).nanoseconds > (self.connection_timeout * 1e9):
            self.get_logger().debug("timeout")
            if self.current_state != State.WAITING_CONNECTION:
                self.get_logger().warn("Connection Lost!")
            self.current_state = State.WAITING_CONNECTION
            self.game_controller = None

        if self.current_state == State.WAITING_CONNECTION:
            self.wait_state()
        elif self.current_state == State.START:
            self.start_state()
        elif self.current_state == State.POSITION_START:
            self.position_start_state()
        elif self.current_state == State.SET:
            self.set_state()
        elif self.current_state == State.KICKOFF:
            self.kickoff_state()
        elif self.current_state == State.PLAYING:
            self.playing_state()
        elif self.current_state == State.FINISH:
            self.finish_state()
        elif self.current_state == State.IDLE:
            self.idle_state()
        elif self.current_state == State.ERROR:
            self.error_state()

    def wait_state(self):
        self.get_logger().info("WAITING_CONNECTION waiting for Game Controller conection")

        if self.game_controller is not None:
            # If reconnecting, return to previous status.
            if self.last_available_state != State.WAITING_CONNECTION:
                self.get_logger().debug("Reconnecting")
                self.current_state = self.last_available_state
            # If not, means is the first connection
            else:
                self.current_state = State.START
                self.last_available_state = self.current_state

    def start_state(self):
        self.get_logger().info("START_STATE waiting for ready from Game Controller")
        if self.game_controller and self.game_controller.game_state == "STATE_READY":
            self.position_start_enable_publisher.publish(Bool(data = True))
            self.current_state = State.POSITION_START
            self.last_available_state = self.current_state
        else:
            self.current_state = State.START

    def position_start_state(self):
        self.get_logger().info("POSITION_START_STATE walking to my position")
        if self.position_start or (self.game_controller and self.game_controller.game_state == "STATE_SET"):
            self.current_state = State.SET
        else:
            self.current_state = State.POSITION_START
    
    def set_state(self):
        self.get_logger().info("SET_STATE waiting for the referee to start the game")
        if self.game_controller and self.game_controller.game_state == "STATE_PLAYING":
            self.current_state = State.KICKOFF
            self.last_available_state = self.current_state
        else:
            self.current_state = State.SET
    def kickoff_state(self):
        self.get_logger().info("KICKOFF_STATE Let's play")
        if self.goalkeeper:
            self.current_state = State.PLAYING
            self.last_available_state = self.current_state
            
        self.team_array_pos =0
        for team in self.game_controller.teams:
            if team.team_number == self.team_number:
                self.target_team = team
                break
        #TODO search a new way to decide who goes for the ball
        if self.player_number == 2:
            print(f"Palyer{self.player_number} going for the kick off")
            self.current_state = State.PLAYING
            self.last_available_state = self.current_state
        else:
            print("waiit for ball moving or pass the time")
            if self.move_ball or self.game_controller.secondary_seconds_remaining==0:
                self.current_state = State.PLAYING
                self.last_available_state = self.current_state
            else:
                self.current_state = State.KICKOFF
    
    def playing_state(self):
        self.get_logger().info("PLAYING_STATE follow ball or goalkeeper guard")
        if self.goalkeeper:
            print("goal keeping") #TODO goal keeper guard enable publisher
        print("Following the ball") #TODO ball follower enable and a way to not crash ones with others

        self.head_ball_follower_enable_publisher.publish(Bool(data = True))
        self.ball_follower_enable_publisher.publish(Bool(data = True))

        if self.game_controller.game_state == "STATE_FINISHED":
            self.get_logger().info("FINISH_STATE good half game")
            self.current_state = State.FINISH
            self.ball_follower_enable_publisher.publish(Bool(data = False))
            self.head_ball_follower_enable_publisher.publish(Bool(data = False))
            self.last_available_state = self.current_state
        


    def finish_state(self):
        self.get_logger().info("THE END going with team")      
        #Maybe TODO go to the own half and out of the field  

    
    def idle_state(self):
        self.get_logger().info("IDLE_STATE new states comming soon")
        self.current_state = State.IDLE

    def error_state(self):
        self.get_logger().error("error_state")
        self.current_state = State.IDLE


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
