#!/usr/bin/env python3

import rclpy
import math
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    from booster_interface.srv import RpcService
except ImportError:
    # En caso de que el paquete no esté en el workspace actual para el autocompletado
    pass

try:
    from social_vision_msgs.msg import VisionObject
except ImportError:
    pass

def get_yaw_from_quaternion(q):
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def shortest_angular_distance(from_angle, to_angle):
    diff = (to_angle - from_angle + math.pi) % (2 * math.pi) - math.pi
    return diff

class State(Enum):
    IDLE = 0
    APPROACHING = 1
    GREETING = 2
    INTERACTING = 3
    RETURN_TURN = 4
    RETURNING = 5
    MARKER_TURN_180 = 6
    MARKER_SCAN_HEAD = 7

class GreetAndReturnSM(Node):
    def __init__(self):
        super().__init__('greet_and_return_sm')
        
        self.state = State.IDLE
        self.home_pose = None
        self.current_pose = None
        self.last_face_time = None
        self.current_distance = 999.0
        
        # Parámetro para seleccionar el método de retorno
        self.declare_parameter("return_method", "marker") # "odometry", "marker", "memory"
        self.return_method = self.get_parameter("return_method").value
        
        # Parámetros de navegación para el retorno
        self.kp_lin = 0.5
        self.kp_yaw = 1.0
        self.max_lin = 0.2
        self.max_ang = 0.6
        
        self.return_yaw_target = 0.0
        self.marker_x = None
        self.marker_dist = 999.0
        self.last_marker_time = None
        
        # Temporizadores y variables de escaneo
        self.greeting_start_time = None
        self.turn_start_time = None
        self.scan_pan = 0.0
        self.scan_tilt = 0.0
        self.search_turn_direction = 1.0
        
        self.command_memory = []
        self.current_cmd_vel = Twist()

        # Publicadores
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_enable_follower = self.create_publisher(Bool, '/person_follower/enable', 10)
        self.pub_head = self.create_publisher(Float32MultiArray, '/hardware/head/goal_pose', 10)
        
        # Suscriptores
        self.sub_odom = self.create_subscription(Odometry, '/odometer_state', self.odom_callback, 10)
        self.sub_distance = self.create_subscription(Float32, '/person_follower/distance', self.distance_callback, 10)
        self.sub_cmd_vel_record = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        try:
            self.sub_marker = self.create_subscription(VisionObject, '/vision/marker', self.marker_callback, 10)
        except NameError:
            self.get_logger().warn("social_vision_msgs no encontrado. Asegúrate de compilarlo.")
        
        # Cliente de Servicio
        self.srv_client = self.create_client(RpcService, '/booster_rpc_service')
        
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("Máquina de Estados Iniciada. Esperando rostros en IDLE...")

    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def cmd_vel_callback(self, msg: Twist):
        self.current_cmd_vel = msg

    def distance_callback(self, msg: Float32):
        self.current_distance = msg.data
        self.last_face_time = self.get_clock().now()

    def marker_callback(self, msg):
        self.marker_x = msg.x
        self.marker_dist = msg.pose.position.x
        self.last_marker_time = self.get_clock().now()

    def enable_person_follower(self, enable: bool):
        msg = Bool()
        msg.data = enable
        self.pub_enable_follower.publish(msg)

    def call_greeting_service(self, hand_action=0):
        if not self.srv_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Servicio /booster_rpc_service no disponible, saltando saludo.")
            return
            
        req = RpcService.Request()
        req.msg.api_id = 2005
        req.msg.body = f'{{"hand_index":1,"hand_action":{hand_action}}}'
        
        future = self.srv_client.call_async(req)
        future.add_done_callback(self.greeting_response_callback)
        
        action_str = "iniciar" if hand_action == 0 else "detener"
        self.get_logger().info(f"Llamada al servicio para {action_str} el saludo enviada.")

    def greeting_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Saludo ejecutado con éxito por el hardware.")
        except Exception as e:
            self.get_logger().error(f"Fallo en el servicio de saludo: {e}")

    def state_machine_loop(self):
        now = self.get_clock().now()
        
        # Evaluar tiempo desde la última vez que se vio el rostro
        time_since_last_face = 999.0
        if self.last_face_time is not None:
            time_since_last_face = (now - self.last_face_time).nanoseconds * 1e-9

        if self.state == State.IDLE:
            # Si detectamos un rostro fresco (hace menos de 0.5 seg)
            if time_since_last_face < 0.5:
                if self.return_method == 'odometry':
                    if self.current_pose is None:
                        self.get_logger().warn("Rostro detectado, pero esperando tópico /odometer_state para guardar Home...", throttle_duration_sec=2.0)
                    else:
                        self.home_pose = self.current_pose
                        self.state = State.APPROACHING
                        self.command_memory.clear() # Limpiar memoria de comandos vieja
                        self.enable_person_follower(True)
                        self.get_logger().info("Persona detectada. Guardando Home (odometría) y acercándose...")
                else:
                    # Si es marker o memory, no necesitamos odometría en lo absoluto
                    self.state = State.APPROACHING
                    self.command_memory.clear()
                    self.enable_person_follower(True)
                    self.get_logger().info(f"Persona detectada. Acercándose (Retorno configurado por {self.return_method})...")

        elif self.state == State.APPROACHING:
            # Si perdemos a la persona antes de llegar, regresamos a casa
            if time_since_last_face > 2.0:
                self.state = State.RETURNING
                self.enable_person_follower(False)
                self.get_logger().info("Se perdió a la persona durante el acercamiento. Regresando...")
                return
                
            # Grabar los comandos que publica el seguidor a 10 Hz
            self.command_memory.append((self.current_cmd_vel.linear.x, self.current_cmd_vel.angular.z))

            # Asumimos que si estamos cerca de 0.5m (tolerancia 0.65m), hemos llegado
            if self.current_distance < 0.65:
                self.state = State.GREETING
                self.enable_person_follower(False) # Detenemos el seguidor
                self.call_greeting_service(0)
                self.greeting_start_time = now
                self.get_logger().info("Llegamos a la persona. Saludando (5 segundos)...")

        elif self.state == State.GREETING:
            # Detener el robot completamente
            self.pub_cmd_vel.publish(Twist())
            elapsed = (now - self.greeting_start_time).nanoseconds * 1e-9 if self.greeting_start_time else 0.0
            if elapsed >= 5.0:
                self.call_greeting_service(1)
                self.state = State.INTERACTING
                self.get_logger().info("Saludo terminado. Interactuando. Esperando a que la persona se vaya...")

        elif self.state == State.INTERACTING:
            # Si la persona se da la vuelta o se aleja (no se detecta su rostro por 3 segundos)
            if time_since_last_face > 3.0:
                if self.return_method == 'marker':
                    self.state = State.MARKER_TURN_180
                    self.turn_start_time = now
                    self.get_logger().info("La persona se ha ido. Dando la vuelta 180 grados...")
                elif self.return_method == 'memory':
                    self.state = State.RETURNING
                    self.get_logger().info("La persona se ha ido. Retornando de reversa usando memoria...")
                    msg_head = Float32MultiArray()
                    msg_head.data = [0.0, 0.0]
                    self.pub_head.publish(msg_head)
                else: # odometry
                    self.state = State.RETURN_TURN
                    if self.home_pose is not None and self.current_pose is not None:
                        dx = self.home_pose.position.x - self.current_pose.position.x
                        dy = self.home_pose.position.y - self.current_pose.position.y
                        self.return_yaw_target = math.atan2(dy, dx)
                    elif self.current_pose is not None:
                        current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
                        self.return_yaw_target = (current_yaw + math.pi) % (2 * math.pi)
                        if self.return_yaw_target > math.pi:
                            self.return_yaw_target -= 2 * math.pi
                    self.get_logger().info("La persona se ha ido. Girando hacia Home por odometría.")

        elif self.state == State.MARKER_TURN_180:
            time_since_marker = (now - self.last_marker_time).nanoseconds * 1e-9 if self.last_marker_time else 999.0
            if time_since_marker < 0.5 and self.marker_x is not None:
                self.state = State.RETURNING
                self.search_turn_direction = 1.0
                self.get_logger().info("Marcador encontrado durante el giro. Retornando...")
                return

            elapsed = (now - self.turn_start_time).nanoseconds * 1e-9 if self.turn_start_time else 0.0
            twist = Twist()
            if elapsed < 6.28: # 3.14 rad a 0.5 rad/s = ~6.28 seg
                twist.angular.z = 0.5
                self.pub_cmd_vel.publish(twist)
            else:
                self.pub_cmd_vel.publish(Twist())
                self.state = State.MARKER_SCAN_HEAD
                self.scan_pan = -1.0 # Empieza mirando a la derecha
                self.scan_tilt = 0.0
                self.get_logger().info("Giro de 180 completado. Escaneando lentamente con la cabeza...")

        elif self.state == State.MARKER_SCAN_HEAD:
            time_since_marker = (now - self.last_marker_time).nanoseconds * 1e-9 if self.last_marker_time else 999.0
            if time_since_marker < 0.5 and self.marker_x is not None:
                self.state = State.RETURNING
                self.search_turn_direction = -1.0 if self.scan_pan < 0 else 1.0
                self.get_logger().info("Marcador encontrado. Centrando cabeza y acercándose...")
                msg_head = Float32MultiArray()
                msg_head.data = [0.0, 0.0]
                self.pub_head.publish(msg_head)
                return

            self.scan_pan += 0.025
            if self.scan_pan > 1.0:
                self.state = State.RETURNING
                self.search_turn_direction = 1.0
                self.get_logger().info("Escaneo de cabeza finalizado sin éxito. Rotando base lentamente para buscar...")
                msg_head = Float32MultiArray()
                msg_head.data = [0.0, 0.0]
                self.pub_head.publish(msg_head)
                return

            msg_head = Float32MultiArray()
            msg_head.data = [self.scan_pan, self.scan_tilt]
            self.pub_head.publish(msg_head)

        elif self.state == State.RETURN_TURN:
            if self.current_pose is None:
                return

            current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
            yaw_err = shortest_angular_distance(current_yaw, self.return_yaw_target)

            if abs(yaw_err) > 0.15:
                twist = Twist()
                twist.angular.z = self.clamp(self.kp_yaw * yaw_err, -self.max_ang, self.max_ang)
                self.pub_cmd_vel.publish(twist)
            else:
                self.pub_cmd_vel.publish(Twist())
                self.state = State.RETURNING
                self.get_logger().info("Giro completado. Iniciando retorno...")
                
                # Acomodar la cabeza para visión
                msg_head = Float32MultiArray()
                msg_head.data = [0.0, 0.0] # Mirar al frente
                self.pub_head.publish(msg_head)

        elif self.state == State.RETURNING:
            if self.return_method == 'odometry':
                if self.home_pose is None or self.current_pose is None:
                    self.state = State.IDLE
                    return

                dx = self.home_pose.position.x - self.current_pose.position.x
                dy = self.home_pose.position.y - self.current_pose.position.y
                dist = math.hypot(dx, dy)

                target_yaw = math.atan2(dy, dx)
                current_yaw = get_yaw_from_quaternion(self.current_pose.orientation)
                home_yaw = get_yaw_from_quaternion(self.home_pose.orientation)

                twist = Twist()

                if dist > 0.15:
                    yaw_err = shortest_angular_distance(current_yaw, target_yaw)
                    twist.angular.z = self.clamp(self.kp_yaw * yaw_err, -self.max_ang, self.max_ang)
                    
                    if abs(yaw_err) < 0.5:
                        twist.linear.x = self.clamp(self.kp_lin * dist, 0.05, self.max_lin)
                else:
                    yaw_err = shortest_angular_distance(current_yaw, home_yaw)
                    if abs(yaw_err) > 0.1:
                        twist.angular.z = self.clamp(self.kp_yaw * yaw_err, -self.max_ang, self.max_ang)
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        self.state = State.IDLE
                        self.get_logger().info("Regreso completado. Esperando nueva persona.")
                        return

                self.pub_cmd_vel.publish(twist)

            elif self.return_method == 'marker':
                twist = Twist()
                time_since_marker = (now - self.last_marker_time).nanoseconds * 1e-9 if self.last_marker_time else 999.0
                
                if time_since_marker < 0.5 and self.marker_x is not None:
                    # Centrar con el marcador visual
                    error_yaw = -(self.marker_x - 320.0) / 320.0 # Asumiendo resolución X 640
                    twist.angular.z = self.clamp(self.kp_yaw * error_yaw, -self.max_ang, self.max_ang)
                    
                    if self.marker_dist > 0.5: # Detenerse a medio metro del marcador
                        twist.linear.x = self.clamp(self.kp_lin * self.marker_dist, 0.05, self.max_lin)
                    else:
                        self.pub_cmd_vel.publish(Twist())
                        self.state = State.IDLE
                        self.get_logger().info("Llegó al marcador. Retorno completado.")
                        return
                else:
                    # Buscar girando mas lento dependiendo de dónde estaba mirando la cabeza
                    twist.angular.z = 0.15 * self.search_turn_direction
                
                self.pub_cmd_vel.publish(twist)

            elif self.return_method == 'memory':
                if len(self.command_memory) > 0:
                    v, w = self.command_memory.pop()
                    twist = Twist()
                    twist.linear.x = -v # Invertir velocidad para caminar hacia atrás
                    twist.angular.z = w # Mantener el mismo giro para deshacer la curva
                    self.pub_cmd_vel.publish(twist)
                else:
                    self.pub_cmd_vel.publish(Twist())
                    self.state = State.IDLE
                    self.get_logger().info("Retorno por memoria de comandos completado.")
                    return

def main(args=None):
    rclpy.init(args=args)
    node = GreetAndReturnSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()