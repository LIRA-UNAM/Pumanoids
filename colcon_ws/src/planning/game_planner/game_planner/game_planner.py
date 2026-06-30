#!/usr/bin/env python3

# Node: game_planner
# The general state machine node to receive Game Controller messages and manage the robot behavior.
# Written by Camile Frias and Sebastian Garcia.
# Developed at LIRA UNAM.
# https://lira.unam.mx/

import math
import rclpy
import socket
import json
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.node import Node
from tf2_ros import TransformException, LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from game_planner import gamestate
from enum import Enum
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D, PoseStamped
from booster_interface.srv import RpcService

# Ports to comunicate with the Game Controller under UDP packages
SOURCE_PORT = 3838 # Game Controller broadcast messages through port 3838
DESTINATION_PORT = 3939 # This node sends messages to Game Controller through port 3939

# --- RETURN_MSGS ---
PENALISE=                    0
UNPENALISE=                  1
ALIVE=                       2
GOALKEEPER=                  3
INTERRUPTION_READY=          4

# --- PENALTY STATES ---
class Penalty(Enum):

    PENALTY_NONE =                      0
    PENALTY_ILLEGAL_POSITIONING =       1
    PENALTY_MOTION_IN_SET =             2
    PENALTY_MOTION_IN_STOP =            3
    PENALTY_LOCAL_GAME_STUCK =          4
    PENALTY_INCAPABLE_ROBOT =           5
    PENALTY_PICK_UP =                   6
    PENALTY_BALL_HOLDING =              7
    PENALTY_LEAVING_THE_FIELD =         8
    PENALTY_PLAYING_WITH_ARMS_HANDS =   9
    PENALTY_PUSHING =                   10
    PENALTY_CAUTIONED =                 11
    PENALTY_SENT_OFF =                  12
    PENALTY_SUBSTITUTE =                13

# --- STATES OF THE NODE ---
class State(Enum):

    # Main states
    
    WAITING_CONNECTION=             0 # Not receiving any signal from the Game Controller
    STATE_INITIAL=                  1 # The initial state of Game Controller
    STATE_READY=                    2 # The robots move from the side to their kickoff positions
    STATE_SET=                      3 # Game controller STATE_SET. Robots must not move in this state.
    STATE_PLAYING=                  4 # Main game state. Play or goalkeep.
    STATE_FINISHED=                 5 # The end
    UNKNOWN=                        6

    # Debug states

    IDLE =                          7 # IDLE
    ERROR =                         8 # Not good
    # More comming soon

# --- GAME PHASE --- 
class Phase(Enum):
    
    # Sub states

    GAME_PHASE_NORMAL=              0
    GAME_PHASE_PENALTY_SHOOT_OUT=   1
    GAME_PHASE_EXTRA_TIME=          2
    GAME_PHASE_TIMEOUT=             3
    
# --- SET PLAY ---
class Play(Enum):
    SET_PLAY_NONE=                  0
    SET_PLAY_DIRECT_FREE_KICK=      1
    SET_PLAY_INDIRECT_FREE_KICK=    2
    SET_PLAY_PENALTY_KICK=          3
    SET_PLAY_THROW_IN=              4
    SET_PLAY_GOAL_KICK=             5
    SET_PLAY_CORNER_KICK=           6


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
        self.declare_parameter('start_position', [0.0, 1.0, -math.pi/2])
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

        # Hysteresis values
        self.declare_parameter('theta_enter_follow', 0.8)
        self.declare_parameter('theta_exit_follow', 1.5)
        self.declare_parameter('ball_min_distance', 0.3)
	
        # Hysteresis + debounce for FOLLOW BALL vs JOELIAN POINT decision   
        self.follow_ball_mode = False   
        self.theta_enter_follow = self.get_parameter('theta_enter_follow').value 
        self.theta_exit_follow = self.get_parameter('theta_exit_follow').value 
        self.mode_switch_counter = 0    
        self.mode_switch_to_joelian_counter = 0    
        self.mode_switch_ticks_required = 3     # ~0.5s sostenidos antes de cambiar de modo 
        self.mode_switch_to_joelian_ticks_required = 30     # ~0.5s sostenidos antes de cambiar de modo 
        # m: si está más cerca que esto, forzar follow ball
        self.close_to_ball_distance = self.get_parameter('ball_min_distance').value

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

        self.goalkeeper_enable_publisher = self.create_publisher(
                Bool,
                '/goal_keeper/enable',          # Or any name of the topic 
                qos_profile_for_enabling)

        self.go_to_target_enable_publisher = self.create_publisher(
                Bool,
                '/go_to_target/enable',
                qos_profile_for_enabling)

        self.target_position_publisher = self.create_publisher(
                Pose2D,
                '/go_to_target/target',
                1)
        self.posestamped_pub = self.create_publisher(
                PoseStamped,
                '/joelian_stamped',
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
        self.getup_req.msg.body = ''

        self.prep_req = RpcService.Request() # Mensaje de request para prep mode 
        self.prep_req.msg.api_id = 2000
        self.prep_req.msg.body = '{"mode":1}'

        self.walk_req = RpcService.Request() # Mensaje de request para walk mode 
        self.walk_req.msg.api_id = 2000
        self.walk_req.msg.body = '{"mode":2}'
        
        
        self.mode_req = RpcService.Request() # Mensaje de request para saber el mode 
        self.mode_req.msg.api_id = 2017
        self.mode_req.msg.body = ''

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
        self.mode_future = None
        self.robot_mode = None
        self.last_ball_positions = [None] * 10
        self.last_joelian_point = [None] * 10
        self.team_in_array = 0 # Position of our team in the array info of game controller 
        self.connection_timeout = 0.7 # <-- Adjust this to set the connection tolerance in seconds :)
        self.last_time_seeing_ball = self.get_clock().now().nanoseconds/1e9
        self.last_packet_time = self.get_clock().now()
        
        # State machine
        self.current_state = State.WAITING_CONNECTION
        self.last_available_state = State.WAITING_CONNECTION
        
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

    def send_get_mode_request(self):
        if self.mode_future is None:
            self.mode_future = self.rpc_client.call_async(self.mode_req)

    def get_mode(self):
        self.send_get_mode_request()
        if self.mode_future is not None and self.mode_future.done():
            try:
                response = self.mode_future.result()
                data = json.loads(response.msg.body)
                self.robot_mode = data["mode"]
                self.get_logger().debug(
                f"Robot mode: {self.robot_mode}"
                )
            except Exception as e:
                self.get_logger().error(f"RPC error: {e}")

            self.mode_future = None

    def send_getup_request(self):
        self.getup_res = self.rpc_client.call_async(self.getup_req)
    
    # State machine method
    def rustic_smach(self):
        self.send_getup_request() # Check if is fall and getup if so
        self.get_mode() # Check the mode of the robot
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
#                self.get_logger().debug(f"Received data from {addr}")
                # Stores the data.
                self.game_controller = gamestate.GameState.parse(data)
                if self.game_controller.teams[0].teamNumber == self.team_number:
                    self.team_in_array = 0 
                else:
                    self.team_in_array = 1

                self.player_info = self.game_controller.teams[self.team_in_array].players[self.player_number-1] # state of penalty of THIS player
                # Update last connection timestamp.
                self.last_packet_time = self.get_clock().now()

                #Game_controller addres
                gc_ip = addr[0]
                #Message with number and if is or not goalkeeper
                if self.current_robot_position is not None:
                    position = self.current_robot_position
                else:
                    position = [0.0, 0.0, 0.0]
                if self.ball_position is not None:
                    self.ball_pos = self.ball_position
                else:
                    self.ball_pos = [0.0, 0.0]

                return_message = gamestate.ReturnData.build(dict(
                    version=gamestate.GAMECONTROLLER_RETURN_STRUCT_VERSION,
                    playerNum=self.player_number,
                    teamNum=self.team_number,
                    fallen = False,
                    pose = position,
                    ballAge = (self.get_clock().now().nanoseconds/1e9) - self.last_time_seeing_ball, 
                    ball = self.ball_pos

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
            self.get_logger().info(f"Current state{self.current_state}")
            if self.current_state is not State.IDLE:
                self.current_state = State[self.game_controller.state]
                self.current_phase = Phase[self.game_controller.gamePhase]
                self.current_play = Play[self.game_controller.setPlay]
                self.current_penalty = Penalty[self.game_controller.teams[self.team_in_array].players[self.player_number -1].penalty]
                self.last_available_state = self.current_state
        
        if self.game_controller and self.current_penalty != Penalty.PENALTY_NONE:
            self.idle_state()
        elif self.game_controller and self.current_phase == Phase.GAME_PHASE_PENALTY_SHOOT_OUT:
            self.penalty_shoot()
        elif self.game_controller and self.current_phase == Phase.GAME_PHASE_TIMEOUT:
            self.idle_state()
        elif self.game_controller and self.current_play != Play.SET_PLAY_NONE:
            self.kick()
        elif self.game_controller and (self.current_phase == Phase.GAME_PHASE_NORMAL or self.current_phase == Phase.GAME_PHASE_EXTRA_TIME) and self.current_play == Play.SET_PLAY_NONE: # Estados principales y primarios del juego
            if self.robot_mode is not None and self.robot_mode == 1:
                walk_res = self.rpc_client.call_async(self.walk_req)
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

    def go_to_target(self, point):
            if self.current_robot_position is None:
                self.get_logger().info("Not pose yet")
                return
            else:
                #distance = math.sqrt((math.pow((self.current_robot_position[0] - point.x),2)+math.pow((self.current_robot_position[1] - point.y),2)))
                #self.get_logger().info(f"Distancia a joeliano: {distance:.2f}m")
                
                # Dejamos que publique siempre que esté habilitado.
                # El nodo de control de bajo nivel manejará la tolerancia de llegada.
                #self.get_logger().info(f"going to {point}")
                #if self.go_to_target_success:
                #   self.target_position_publisher.publish(point)
                self.target_position_publisher.publish(point)

    def wait_state(self):
        self.get_logger().info("WAITING_CONNECTION waiting for Game Controller conection")

    def start_state(self):
        self.get_logger().info("START_STATE waiting for ready from Game Controller")
        self.go_to_target_enable_publisher.publish(Bool(data = False))
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.ball_follower_enable_publisher.publish(Bool(data = False))

    def ready_state(self):
        self.get_logger().info("READY_STATE going to my initial position")
        self.go_to_target_enable_publisher.publish(Bool(data = True))
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.ball_follower_enable_publisher.publish(Bool(data = False))
        self.go_to_target(self.start_position)

    def set_state(self):
        self.get_logger().info("SET_STATE waiting for the referee to start the game")
        if self.game_controller.secondaryTime > 0 and self.game_controller.kicking != self.team_number:
            self.get_logger().info("Waiting for the kickoff")
            self.go_to_target_enable_publisher.publish(Bool(data = False))
            self.head_ball_follower_enable_publisher.publish(Bool(data = False))
            self.ball_follower_enable_publisher.publish(Bool(data = False))
            return
        else:
            self.go_to_target_enable_publisher.publish(Bool(data = False))
            self.head_ball_follower_enable_publisher.publish(Bool(data = True))
            self.ball_follower_enable_publisher.publish(Bool(data = True))


    
    def playing_state(self):
        if self.goalkeeper == True:
            self.goalkeeper_enable_publisher.publish(Bool(data = True))
            self.get_logger().info("Goal_keeping")
            return 
        self.head_ball_follower_enable_publisher.publish(Bool(data=True))
        if self.carry_ball_position is not None and self.ball_position is not None:

            joelian_point = [self.carry_ball_position[0], self.carry_ball_position[1]]
            robot_position = [self.current_robot_position[0], self.current_robot_position[1]]

            # CORRECCIÓN MATEMÁTICA VITAL:
            # V1: Vector desde el Punto Joeliano HASTA la Pelota (Vector de Ataque ideal)
            V1 = [self.ball_position[0] - joelian_point[0], self.ball_position[1] - joelian_point[1]]
            # V2: Vector desde el Robot HASTA la Pelota (Vector de Ataque real)
            V2 = [self.ball_position[0] - robot_position[0], self.ball_position[1] - robot_position[1]]

            M1 = math.sqrt(V1[0] * V1[0] + V1[1] * V1[1])
            M2 = math.sqrt(V2[0] * V2[0] + V2[1] * V2[1])

            # Evitar divisiones por cero si hay un error de visión extremo
            if M1 == 0 or M2 == 0:
                self.get_logger().info(f"Zeros")
                wants_follow = False
            # --- Caso especial: muy cerca de la pelota, forzar persecución ---
            #elif M2 < self.close_to_ball_distance:
            #    wants_follow = True
            else:
                cos_theta = max(-1.0, min(1.0, (V1[0] * V2[0] + V1[1] * V2[1]) / (M1 * M2)))
                theta = math.acos(cos_theta)
                self.get_logger().info(f"Angle = {theta:.2f} rad")

                if self.follow_ball_mode:
                    if (theta > self.theta_exit_follow):
                        wants_follow = False
                    else:
                        wants_follow = True
                else:
                    # Si el ángulo entre nuestro ataque real y el ideal es menor a ~40 grados, ataca.
                    wants_follow = (theta < self.theta_enter_follow)

            self.follow_ball_mode = wants_follow

            if self.follow_ball_mode:
                self.get_logger().info("FOLLOW BALL")
                self.go_to_target_enable_publisher.publish(Bool(data=False))
                self.head_ball_follower_enable_publisher.publish(Bool(data=True))
                self.ball_follower_enable_publisher.publish(Bool(data=True))
            else:
                self.get_logger().info("JOELIAN POINT")
                self.ball_follower_enable_publisher.publish(Bool(data=False))
                self.head_ball_follower_enable_publisher.publish(Bool(data=True))
                self.go_to_target_enable_publisher.publish(Bool(data=True))
                target = Pose2D()
                target.x = self.carry_ball_position[0]
                target.y = self.carry_ball_position[1]
                target.theta = self.carry_ball_position[2]
                #self.get_logger().debug(f"Robot  a: {self.current_robot_position[2]}")
                #self.get_logger().debug(f"Target a: {self.current_robot_position[2]}")
                self.go_to_target(target)

    def finish_state(self):
        self.get_logger().info("FINISH_STATE good half game")
        self.ball_follower_enable_publisher.publish(Bool(data = False))
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.get_logger().info("THE END going with team")    
        if self.target_arrive_success:
            self.go_to_target(self.start_position)

    def penalty_shoot(self):
        self.get_logger().info("PENALTYSHOOT sub state")
        if self.game_controller.kickingTeam == self.team_number:
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

    def throwin(self):
        self.get_logger().info("Truely not idea of what this is ")

    def idle_state(self):
        self.go_to_target_enable_publisher.publish(Bool(data = False)) 
        self.head_ball_follower_enable_publisher.publish(Bool(data = False))
        self.ball_follower_enable_publisher.publish(Bool(data = False))
        self.get_logger().debug(f"Las state aviable {self.last_available_state}")
        if self.last_available_state is not State.IDLE and self.last_available_state is not State.WAITING_CONNECTION: 
            self.counter = 0 
        else:
            #self.get_logger().debug("counter + 1")
            self.counter+=1
        if self.counter>=20:
            prep_res = self.rpc_client.call_async(self.prep_req)
            self.get_logger().debug(f"RPC Service response: {prep_res}")
        if self.game_controller and (self.player_info.penalty!=Penalty.PENALTY_NONE):
            self.last_available_state = self.current_state
            self.current_state = State.IDLE
        else:
            self.last_available_state = self.current_state
            self.current_state = State[self.game_controller.game_state]

        self.get_logger().info(f"IDLE_STATE {self.counter}")

    def error_state(self):
        self.get_logger().error("error_state")
        self.current_state = State.IDLE

    def pose2d_to_timestamped(self, msg: Pose2D):
        stamped_msg = PoseStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = "pumas_map"

        stamped_msg.pose.position.x = msg.x
        stamped_msg.pose.position.y = msg.y
        stamped_msg.pose.position.z = 0.0

        stamped_msg.pose.orientation.x = 0.0
        stamped_msg.pose.orientation.y = 0.0
        stamped_msg.pose.orientation.z = math.sin(msg.theta / 2.0)
        stamped_msg.pose.orientation.w = math.cos(msg.theta / 2.0)

        self.posestamped_pub.publish(stamped_msg)

    # --- CALLBACKS ---
    def map_ball_callback(self, msg):
        self.last_ball_positions.pop(0)
        self.last_ball_positions.append([msg.x, msg.y])
        
        valid = [pos for pos in self.last_ball_positions if pos is not None]
        if not valid:
            return
        avg_x = sum(p[0] for p in valid) / len(valid)
        avg_y = sum(p[1] for p in valid) / len(valid)
        self.ball_position = [avg_x, avg_y]
        self.last_time_seeing_ball = self.get_clock().now().nanoseconds/1e9
        
        #x_mean = 0.0
        #y_mean = 0.0
        #for i in self.last_ball_positions:
        #    x_mean += i[0]
        #    y_mean += i[1]
        #x_mean = x_mean/10
        #y_mean = y_mean/10
        #self.ball_position = [x_mean, y_mean]

    def carry_ball_callback(self, msg):
        self.last_joelian_point.pop(0)
        self.last_joelian_point.append([msg.x, msg.y, msg.theta])

        valid = [pos for pos in self.last_joelian_point if pos is not None]
        if not valid:
            return
        avg_x = sum(p[0] for p in valid) / len(valid)
        avg_y = sum(p[1] for p in valid) / len(valid)
        avg_theta = sum(p[2] for p in valid) / len(valid)
        self.carry_ball_position = [avg_x, avg_y, avg_theta]

        rviz_joelian = Pose2D()
        rviz_joelian.x = avg_x
        rviz_joelian.y = avg_y
        rviz_joelian.theta = avg_theta
        self.pose2d_to_timestamped(rviz_joelian)

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
   
