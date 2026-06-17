#!/usr/bin/env python3

# Node: game_planner
# The general state machine node to receive Game Controller messages and manage the robot behavior.
# Written by Camile Frias and Sebastian Garcia.
# Developed at LIRA UNAM.
# https://lira.unam.mx/

import math
import rclpy
import socket
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from tf2_ros import TransformException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from game_planner import gamestate
from enum import Enum
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from booster_interface.srv import RpcService

# Ports to comunicate with the Game Controller under UDP packages
SOURCE_PORT = 3838 # Game Controller broadcast messages through port 3838
DESTINATION_PORT = 3939 # This node sends messages to Game Controller through port 3939
COACH_PORT = 3839 # To communicate with the coach

# --- RETURN_MSGS ---
PENALISE=                    0
UNPENALISE=                  1
ALIVE=                       2
GOALKEEPER=                  3
INTERRUPTION_READY=          4


# --- STATES OF THE NODE ---
class State(Enum):

    # Main states
    
    WAITING_CONNECTION= 0 # Not receiving any signal from the Game Controller
    STATE_INITIAL=      1 # The initial state of Game Controller
    STATE_READY=        2 # The robots move from the side to their kickoff positions
    STATE_SET=          3 # Game controller STATE_SET. Robots must not move in this state.
    STATE_PLAYING=      4 # Main game state. Play or goalkeep.
    STATE_FINISHED=     5 # The end

    # Sub states

    STATE_NORMAL=               6
    STATE_PENALTYSHOOT=         7
    STATE_OVERTIME=             8
    STATE_TIMEOUT=              9
    STATE_DIRECT_FREEKICK=      10
    STATE_INDIRECT_FREEKICK=    11
    STATE_PENALTYKICK=          12
    STATE_CORNERKICK=           13
    STATE_GOALKICK=             14
    STATE_THROWIN=15
    DROPBALL=16
    UNKNOWN=17

    # Debug states

    IDLE = 18 # IDLE
    ERROR = 19 # Not good
    # More comming soon

class PlannerNode(Node):
    def __init__(self):
        super().__init__('game_planner')
        self.get_logger().info('game_planner iniciado. Esperando...') 
        self.get_logger().info('Ctrl+C para cerrar.')
        
        # --- PARAMETERS ---
        # robot_model
        self.declare_parameter('robot_model', 'k1')
        self.robot_model_ = self.get_parameter('robot_model').get_parameter_value().string_value
        if self.robot_model_ == 'k1' or self.robot_model_ == 't1':
            self.get_logger().info(f"Selected robot: {self.robot_model_}")
        else:
            self.get_logger().error(f"Unknown robot model: {self.robot_model_}")

        # player_number
        self.declare_parameter('player_number', 1)
        self.player_number = self.get_parameter('player_number').value
        self.get_logger().info(f'Jugador {self.player_number} Listo')

        # start_position 
        self.declare_parameter('start_position', [-2.0, -4.0, math.pi/2])
        self.start_position = Pose2D()
        self.start_position.x = self.get_parameter('start_position').value[0]
        self.start_position.y = self.get_parameter('start_position').value[1]
        self.start_position.theta = self.get_parameter('start_position').value[2]
        self.get_logger().info(f'posicion inicial(x,y): {self.start_position}')

        # team_number
        self.declare_parameter('team_number', 0)
        self.team_number = self.get_parameter('team_number').value
        self.get_logger().info(f'Equipo número {self.team_number} PUMANOIDS')

        # Is goalkeeper?
        self.declare_parameter('goalkeeper', False)
        self.goalkeeper = self.get_parameter('goalkeeper').value
        self.get_logger().info(f'Portero? {self.goalkeeper} ')

        #Is going to do the kickoff?
        self.declare_parameter('kickoff', True)
        self.kickoff_robot = self.get_parameter('kickoff').value
        self.get_logger().info(f'Irá a por el kickoff? {self.kickoff_robot} ')

        #Is going to do the kick?
        self.declare_parameter('kick', True)
        self.kick_robot = self.get_parameter('kick').value
        self.get_logger().info(f'Irá a por el kick para los subestados que lo necesiten? {self.kick_robot} ')

        # --- QoS PROFILES ---
        qos_profile_for_enabling = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # --- TF2 ---
        self.target_frame = self.declare_parameter('pumas_map', 'pumas_base_link').get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- PUBLISHERS ---
        self.head_ball_follower_enable_publisher = self.create_publisher(
                Bool,
                '/head_ball_follower/enable',
                qos_profile_for_enabling)

        self.ball_follower_enable_publisher = self.create_publisher(
                Bool,
                '/ball_follower/enable',
                qos_profile_for_enabling)

        self.go_to_target_enable_publisher = self.create_publisher(
                Bool,
                '/go_to_target/enable',
                qos_profile_for_enabling)

        self.target_position_publisher = self.create_publisher(
                Pose2D,
                '/go_to_target/target',
                10)
        
        # --- SUBSCRIBERS ---
        # carry_ball_to_goal
        self.carry_ball_to_goal_pose_subscriber = self.create_subscription(
                Pose2D,
                '/carry_ball_to_goal/point',
                self.carry_ball_callback,
                10)
        
        # go_to_target success
        self.target_arrive_success = self.create_subscription(
                Bool,
                '/go_to_target/success',
                self.go_to_target_success_callback,
                10)
        
        # map_ball localization
        self.ball_position_subscriber = self.create_subscription(
                Pose2D,
                '/vision/map_ball',
                self.map_ball_callback,
                10)

        # --- SERVICES CLIENTS ---
        self.rpc_client = self.create_client(RpcService, '/booster_rpc_service')
        while not self.rpc_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio...')
        
        self.getup_req = RpcService.Request() # Mensaje de request para getup, llama al Api ID siempre igual
        self.getup_req.msg.api_id = 2008
        self.getup_req.msg.body = ""

        self.prep_req = RpcService.Request() # Mensaje de request para prep mode 
        self.prep_req.msg.api_id = 2000
        self.prep_req.msg.body = '{"mode":1}'


        # --- TIMERS ---
        # State machine
        self.timer = self.create_timer(0.1, self.rustic_smach)

        # tf2
        self.tf_timer = self.create_timer(0.15, self.tf_callback)

        # --- VARIABLES ---
        # Game Controller
        self.host ="0.0.0.0" # Always watching any IP
        self.move_ball = False
        self.game_controller = None
        self.ball_position = None
        self.target_team = None
        self.team_in_array = 0 # Position of our team in the array info of game controller 
        self.connection_timeout = 0.7 # <-- Adjust this to set the connection tolerance in seconds :)
        self.last_packet_time = self.get_clock().now()
        
        # State machine
        self.current_state = State.WAITING_CONNECTION
        self.sub_state = State.STATE_NORMAL
        self.last_available_state = State.WAITING_CONNECTION
        self.first_message = True
        
        # Positioning
        self.go_to_target_success = True 
        self.current_robot_position = None
        self.carry_ball_position = None # Position from carry_ball_to_goal node

        # --- SOCKETS ---
        #Broadcast from game_controller
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.server_socket.bind((self.host,SOURCE_PORT))
        self.server_socket.setblocking(False)
        
    
    # Service request
    def send_getup_request(self):
        self.getup_res = self.rpc_client.call_async(self.getup_req)
    
    # State machine method
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
  #              self.get_logger().error(f'Error: {e}')
                self.current_state = self.error_state

        try:
            # Loop to get the latest packet and clear the buffer
            # Once the buffer is empty, it will exit the while loop.
            while True:
                # Attempts to read up to 1024 bytes from the UDP socket.
                data, addr = self.server_socket.recvfrom(1024)
#                self.get_logger().debug(f"Received data from {addr}")
                # Stores the data.
                self.game_controller = gamestate.GameState.parse(data)
                if self.first_message:
                    if self.game_controller.teams[0].team_number == self.team_number:
                        self.team_in_array = 0 
                    else:
                        self.team_in_array = 1
                    self.first_message = False

                self.player_info = self.game_controller.teams[self.team_in_array].players[self.player_number-1] # state of penalty of THIS player
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
        
        if self.game_controller and (self.player_info.penalty!=0):
            if self.player_info.penalty == 30: #Ball manipulation
                self.idle_state()
            elif self.player_info.penalty == 31: #pushing
                self.pushing()
            elif self.player_info.penalty == 32: # illegal_atk
                self.illegal_atk()
            elif self.player_info.penalty == 33: # illegal_def
                self.illegal_def()
            elif self.player_info.penalty == 34: # pickup 
                self.idle_state()
            elif self.player_info.penalty == 35: # Service No idea of what
                self.idle_state()
        elif self.game_controller and (self.game_controller.secondary_state == "STATE_NORMAL" or self.game_controller.secondary_state =="STATE_OVERTIME"): # Estados principales y primarios del juego
            if self.current_state == State.WAITING_CONNECTION:
                self.wait_state()
            elif self.current_state == State.STATE_INITIAL:
                self.start_state()
            elif self.current_state == State.STATE_READY:
                self.ready_state()
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
                self.dropball() # NO IDEA of what is this x2 
            elif self.sub_state == State.UNKNOWN:
                self.error_state()

    def go_to_target(self, point):
        if self.current_robot_position is None:
            self.get_logger().info("Not pose yet")
            return
        else:
            distance = math.sqrt((math.pow((self.current_robot_position[0] - point.x),2)+math.pow((self.current_robot_position[1] - point.y),2)))
            send =(self.go_to_target_success  and distance > 0.5) 
            if send:
                self.get_logger().info(f"going to {point}")
                self.target_position_publisher.publish(point)

    def wait_state(self):
        self.get_logger().info("WAITING_CONNECTION waiting for Game Controller conection")

    def start_state(self):
        self.get_logger().info("START_STATE waiting for ready from Game Controller")

    def ready_state(self):
        self.get_logger().info("READY_STATE going to my initial position")
        self.go_to_target(self.start_position)

    def set_state(self):
        self.get_logger().info("SET_STATE waiting for the referee to start the game")
    
    def playing_state(self):
        # TODO check if ball is outside of center 
        if self.game_controller.secondary_seconds_remaining==0 or self.goalkeeper: # TODO insert here the ball outside center comparision
#            self.get_logger().info("PLAYING_STATE follow ball or goalkeeper guard")
            if self.goalkeeper:
                print("goal keeping") #TODO goal keeper guard enable publisher
            else:
                if self.current_robot_position is None:
                    self.get_logger().info("Not pumas map yet")
                    return
                if self.carry_ball_position is not None  and self.ball_position is not None:
                    self.get_logger().info("I've my position and the ball position")

                    # Calculate distante to the joelian point
                    joelian_point = [self.carry_ball_position[0], self.carry_ball_position[1]]
                    robot_position = [self.current_robot_position[0], self.current_robot_position[1]]
                    self.get_logger().debug(f"Dist to joelian point: {math.dist(joelian_point, robot_position)}")

                    # --- CHOOSING BETWEEN GO_TO_TARGET AND BALL_FOLLOWER ---
                    # Calculate a line that goes through ball and goal
                    self.get_logger().info(f"ball position: {self.ball_position}, joelian_point: {joelian_point}, robot_position: {robot_position}")
                    A = joelian_point[1] - self.ball_position[1]
                    B = self.ball_position[0] - joelian_point[0]
                    C = -A * joelian_point[0] - B * joelian_point[1]
                    V1 = [joelian_point[0]- self.ball_position[0], joelian_point[1] - self.ball_position[1]]
                    V2 = [robot_position[0]- self.ball_position[0], robot_position[1]- self.ball_position[1]]
                    M1 = math.sqrt(V1[0] * V1[0] + V1[1] * V1[1])
                    M2 = math.sqrt(V2[0] * V2[0] + V2[1] * V2[1])
                    theta = math.acos((V1[0] * V2[0] + V1[1] * V2[1])/(M1*M2))
                    self.get_logger().info(f"Angle = {theta}")
                    #distancia = abs(A*robot_position[0] + B* robot_position[1] + C)/math.sqrt(A**2 + B**2)
                    #self.get_logger().info(f"Distancia {distancia}")
                    #y = (((joelian_point[1] - self.ball_position[1])/(joelian_point[0]
                    #    - self.ball_position[0]))*(robot_position[0]
                    #    - self.ball_position[0]))+self.ball_position[1]
                    # If the robot is inside that line activate ball follwer 
                    #if abs(y-robot_position[1]) < 10.0 :
                    #if distancia < 1.0:
                    if abs(theta) < 1.0:
                        self.go_to_target_enable_publisher.publish(Bool(data = False))
                        self.head_ball_follower_enable_publisher.publish(Bool(data = True))
                        self.ball_follower_enable_publisher.publish(Bool(data = True))
                    # else go to the target in joelian point.
                    else:
                        target = Pose2D()
                        target.x = self.carry_ball_position[0]
                        target.y = self.carry_ball_position[1]
                        target.theta = self.carry_ball_position[2]
                        self.go_to_target(target)
                    # -------------------------------------------------------
                else:
                    self.get_logger().info("Searching the ball")
                    self.go_to_target_enable_publisher.publish(Bool(data = False))
                    self.head_ball_follower_enable_publisher.publish(Bool(data = True))
                    self.ball_follower_enable_publisher.publish(Bool(data = False))
        else:
            #TODO another way to decied wich robot do the kickoff
            if self.game_controller.kick_of_team == self.team_number and self.kickoff_robot:
                #TODO Miguels kick implementation  
                self.head_ball_follower_enable_publisher.publish(Bool(data = True))
                self.ball_follower_enable_publisher.publish(Bool(data = True))
            else:
                print("waiit for ball moving or pass the time")


    def finish_state(self):
        self.get_logger().info("FINISH_STATE good half game")
        self.ball_follower_enable_publisher.publish(Bool(data = False))
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.get_logger().info("THE END going with team")    
        if self.target_arrive_success:
            self.go_to_target(self.start_position)

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
                #TODO waiting for miguels function to kick 
            if byte_array[1] == 2:
                self.get_logger().info("kicking the ball")
        else:
            #TODO function to not hinder the robot kick or go for the bounce 
            self.get_logger().info("going to another place")

    def illegal_atk(self):
        print("Sorry i attacked before time") #TODO go back to avoid pushing other robot or try to scape the mob
    
    def illegal_def(self):
        print("sorry i get in ur way") #TODO go back to avoid pushing other robot or try to scape the mob
    
    def pushing(self):
        print("yikes need to get out of the mob") #TODO go back to avoid pushing other robot or try to scape the mob

    
    def idle_state(self):
        self.go_to_target_enable_publisher.publish(Bool(data = False)) 
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.ball_follower_enable_publisher.publish(Bool(data = False))
        self.get_logger().info(f"Las state aviable {self.last_available_state}")
        if self.last_available_state != State.IDLE or self.last_available_state != State.WAITING_CONNECTION: 
            self.counter = 0 
        else:
            self.counter+=1
        if self.counter>=20:
            prep_res = self.rpc_client.call_async(self.prep_req)
            self.get_logger().debug(f"RPC Service response: {prep_res}")
        self.current_state = State.IDLE
        self.get_logger().info(f"IDLE_STATE {self.counter}")

    def error_state(self):
        self.get_logger().error("error_state")
        self.current_state = State.IDLE

    # --- CALLBACKS ---
    def map_ball_callback(self, msg):
        self.ball_position = [msg.x, msg.y]

    def carry_ball_callback(self, msg):
        self.carry_ball_position = (msg.x, msg.y, msg.theta)

    def go_to_target_success_callback(self, msg):
        self.go_to_target_success = msg.data

    # To get and store the robot absolute position
    def tf_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'pumas_map',
                'pumas_base_link',
                rclpy.time.Time())
            self.current_robot_position = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    math.atan2(t.transform.rotation.z, t.transform.rotation.w)*2
                    ]
        except (LookupException):
            self.get_logger().info('Waiting for pumas_map->pumas_base_link')
        except TransformException as ex:
            self.get_logger().error(f'TF2 exception: {ex}')


def main(args=None):
    rclpy.init(args=args)

    node = PlannerNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
   
