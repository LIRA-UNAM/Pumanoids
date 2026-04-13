#!/usr/bin/env python3

# Package: game_planner
# Node: game_planner
# The general state machine node to receive Game Controller messages and manage the robot behavior.
# Written by Camile Frias and Sebastian Garcia.
# Developed at LIRA UNAM.
# https://lira.unam.mx/

import rclpy
import socket
from rclpy.node import Node
from game_planner import gamestate
from enum import Enum
from std_msgs.msg import Bool
from booster_interface.srv import RpcService
from carry_ball_to_goal_interfaces.srv import GetGoalRobotPose 

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

        # -- MAIN STATES --
    WAITING_CONNECTION = 0 # Not receiving any signal from the Game Controller
    STATE_INITIAL = 1 # Game controller STATE_READY. The initial state of Game Controller
    STATE_READY = 2 # The robots move from the side to their kickoff positions
    STATE_SET = 3 # Game controller STATE_SET. Robots must not move in this state.
    STATE_PLAYING = 4 # To follow the ball detected by the robot's camrea. If goalkeeper, run the defense node.
    STATE_FINISHED = 5 # THE END

        # -- SUB STATES --

    STATE_NORMAL=6
    STATE_PENALTYSHOOT=7
    STATE_OVERTIME=8
    STATE_TIMEOUT=9
    STATE_DIRECT_FREEKICK=10
    STATE_INDIRECT_FREEKICK=11
    STATE_PENALTYKICK=12
    STATE_CORNERKICK=13
    STATE_GOALKICK=14
    STATE_THROWIN=15
    DROPBALL=16
    UNKNOWN=17

        # -- DEBUG STATES --

    IDLE = 18 # IDLE
    ERROR = 19 # Not good
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

        #Is going to do the kickoff?
        self.declare_parameter('kickoff', False)
        self.kickoff_robot = self.get_parameter('kickoff').value
        self.get_logger().info(f'Irá a por el kickoff? {self.kickoff_robot} ')

        #Is going to do the kick?
        self.declare_parameter('kick', False)
        self.kick_robot = self.get_parameter('kick').value
        self.get_logger().info(f'Irá a por el kick para los subestados que lo necesiten? {self.kick_robot} ')

        # -- SERVICES CLIENTS --

        #Getup service TODO for booster T1
        self.getup_client = self.create_client(RpcService, '/booster_rpc_service')
        while not self.getup_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio...')
        
        self.getup_req = RpcService.Request() # Mensaje de request para getup, llama al Api ID siempre igual
        self.getup_req.msg.api_id = 2008
        self.getup_req.msg.body = ""

        #Ball to gate service 
        self.go_to_gate_client = self.create_client(GetGoalRobotPose, '/get_goal_robot_pose')
        while not self.go_to_gate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio...')
        
        self.go_to_gate_req = GetGoalRobotPose.Request() #Request siempre vacío


        
        # VARIABLES
        self.host ="0.0.0.0" # Always watching any IP
        self.position_start = False
        self.move_ball = False
        self.game_controller = None
        self.target_team = None
        self.connection_timeout = 0.7 # <-- Adjust this to set the connection tolerance in seconds :)
        self.last_packet_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.rustic_smach) # Timer for the rustic_smach function.
        self.current_state = State.WAITING_CONNECTION
        self.sub_state = State.STATE_NORMAL
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
    
    # -- SERVICE REQUEST --

    def send_getup_request(self):
        self.getup_res = self.getup_client.call_async(self.getup_req)
    
    def send_go_to_gate_request(self):
        self.go_to_gate_res = self.client.call_async(self.go_to_gate_req)
        if self.go_to_gate_res.succes:
            self.incident_x =self.go_to_gate_res.pose.position.x
            self.incident_y =self.go_to_gate_res.pose.position.y
            self.incident_z =self.go_to_gate_res.pose.position.z    
            self.incident_z =self.go_to_gate_res.pose.orientation.z
            self.incident_w =self.go_to_gate_res.pose.orientation.w
    
    def rustic_smach(self):
        self.send_getup_request() # Check if is fall and getup if so
        if self.getup_res.done():
            try:
                if self.getup_res.result().msg.status == 0:
                    self.get_logger().debug("Robot levantado")
                else:
                    self.get_logger().debug("Robot no se puede levantar ")
                    self.current_state = self.error_state
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
                self.current_state = self.error_state



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

        #Update state from game controller messages
        if self.game_controller:
            self.current_state = State[self.game_controller.game_state]
            self.last_available_state = self.current_state

            self.sub_state = State[self.game_controller.secondary_state]
            self.last_available_sub_state = self.sub_state

        if self.game_controller and (self.game_controller.secondary_state == "STATE_NORMAL" or self.game_controller.secondary_state =="STATE_OVERTIME"): # Estados principales y primarios del juego
            if self.current_state == State.WAITING_CONNECTION:
                self.wait_state()
            elif self.current_state == State.STATE_INITIAL:
                self.start_state()
            elif self.current_state == State.STATE_READY:
                self.position_start_state()
            elif self.current_state == State.STATE_SET:
                self.set_state()
            elif self.current_state == State.STATE_PLAYING:
                self.playing_state()
            elif self.current_state == State.STATE_FINISHED:
                self.finish_state()
            elif self.current_state == State.IDLE:
                self.idle_state()
            elif self.current_state == State.ERROR:
                self.error_state()

        else: #Estados Secundarios de tiros por faltas y así, nunca he visto uno
            if self.sub_state == State.STATE_PENALTYSHOOT:
                self.penalty_shoot()
            elif self.sub_state == State.STATE_TIMEOUT:
                self.idle_state()
            elif self.sub_state == State.STATE_DIRECT_FREEKICK:
                self.kick()
            elif self.sub_state == State.STATE_INDIRECT_FREEKICK:
                self.kick()
            elif self.sub_state == State.STATE_PENALTYKICK:
                self.kick()
            elif self.sub_state == State.STATE_CORNERKICK:
                self.kick()
            elif self.sub_state == State.STATE_GOALKICK:
                self.kick()
            elif self.sub_state == State.STATE_THROWIN:
                self.throwin() # NO IDEA of what is this
            elif self.sub_state == State.DROPBALL:
                self.dropball() # NO IDEA of what is thisx
            elif self.sub_state == State.UNKNOWN:
                self.error_state()


    def wait_state(self):
        self.get_logger().info("WAITING_CONNECTION waiting for Game Controller conection")

        # if self.game_controller is not None:
        #     # If reconnecting, return to previous status.
        #     if self.last_available_state != State.WAITING_CONNECTION:
        #         self.get_logger().debug("Reconnecting")
        #         self.current_state = self.last_available_state
        #     # If not, means is the first connection
        #     else:
        #         self.current_state = State.STATE_INITIAL
        #         self.last_available_state = self.current_state

    def start_state(self):
        self.get_logger().info("START_STATE waiting for ready from Game Controller")

    def position_start_state(self):
        self.position_start_enable_publisher.publish(Bool(data = True))
        self.get_logger().info("POSITION_START_STATE walking to my position")
    
    def set_state(self):
        self.get_logger().info("SET_STATE waiting for the referee to start the game")
    
    def playing_state(self):

        if self.game_controller.secondary_seconds_remaining==0 or self.goalkeeper:
            self.get_logger().info("PLAYING_STATE follow ball or goalkeeper guard")
            if self.goalkeeper:
                print("goal keeping") #TODO goal keeper guard enable publisher
            else:
                print("Following the ball") #TODO ball follower enable and a way to not crash ones with others

                self.head_ball_follower_enable_publisher.publish(Bool(data = True))
                self.ball_follower_enable_publisher.publish(Bool(data = True))
                #TODO do the arrive to the ball node 
                self.send_go_to_gate_request() # Ball to gate request 
                #TODO do the evade other humanoids or pass ball node

        else:
            #TODO another way to decied wich robot do the kickoff
            if self.game_controller.kick_of_team == self.team_number and self.kickoff_robot:
                #TODO another way to do the kickoff
                self.head_ball_follower_enable_publisher.publish(Bool(data = True))
                self.ball_follower_enable_publisher.publish(Bool(data = True))
            else:
                print("waiit for ball moving or pass the time")


    def finish_state(self):
        self.get_logger().info("FINISH_STATE good half game")
        self.ball_follower_enable_publisher.publish(Bool(data = False))
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.get_logger().info("THE END going with team")      
        #Maybe TODO go to the own half and out of the field  

    def penalty_shoot(self):
        self.get_logger().info("PENALTYSHOOT sub state")
        if self.game_controller.kick_of_team == self.team_number:
            self.get_logger().info("Robot going to shoot")
            if self.current_state == State.STATE_SET:
                self.get_logger().info("Going to designed position")
                #TODO Ir a posición de penal
            elif self.current_state == State.STATE_PLAYING:
                self.get_logger().info("Shooting penal")
                #TODO Tirar penal
        else:
            self.get_logger().info("Goalkeeper must defend")
            if self.current_state == State.STATE_SET:
                self.get_logger().info("Going to designed position")
                #TODO Ir a posición de portería
            elif self.current_state == State.STATE_PLAYING:
                self.get_logger().info("Defending")
                #TODO Defender el penal como pueda

    def kick(self):
        self.get_logger().info("Some of the lots of kicks")
        byte_array = self.game_controller.secondary_state_info
        if byte_array[0] == self.team_number and self.kick_robot:
            #TODO go to the ball that the referee will put after the kick you have execute
            if byte_array[1] == 1:
                self.get_logger().info("Posicioning to the ball")
                self.send_go_to_gate_request() #Same ball to gate request to kick to the goal or try at least TODO find a better function
            if byte_array[1] == 2:
                self.get_logger().info("kicking the ball")
        else:
            #TODO function to not hinder the robot kick or go for the bounce 
            self.get_logger().info("going to another place")

             
    
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



    def wait_state(self):
        self.get_logger().info("WAITING_CONNECTION waiting for Game Controller conection")

        # if self.game_controller is not None:
        #     # If reconnecting, return to previous status.
        #     