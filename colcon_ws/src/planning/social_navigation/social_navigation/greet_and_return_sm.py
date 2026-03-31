#!/usr/bin/env python3

import rclpy
from enum import Enum
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from pumas_vision_msgs.msg import VisionObject

try:
    from booster_interface.srv import RpcService
except ImportError:
    # En caso de que el paquete no esté en el workspace actual para el autocompletado
    pass

class State(Enum):
    IDLE = 0
    APPROACHING = 1
    GREETING = 2
    INTERACTING = 3

class GreetAndReturnSM(Node):
    def __init__(self):
        super().__init__('greet_and_return_sm')
        
        self.state = State.IDLE
        self.last_face_time = None
        self.current_distance = 999.0
        
        self.greeting_start_time = None

        # Publicadores
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_enable_follower = self.create_publisher(Bool, '/person_follower/enable', 10)
        
        # Suscriptores
        self.sub_face = self.create_subscription(VisionObject, '/vision/face', self.face_callback, 10)
        
        # Cliente de Servicio
        self.srv_client = self.create_client(RpcService, '/booster_rpc_service')
        
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("Máquina de Estados Iniciada. Buscando persona en IDLE...")

    def face_callback(self, msg: VisionObject):
        self.current_distance = msg.pose.position.x

        # Si estamos en el estado de interacción, ignoramos cualquier rostro
        # que esté a más de 66cm. Esto evita que el robot se "distraiga"
        # con personas que pasan por detrás.
        if self.state == State.INTERACTING and self.current_distance > 0.66:
            return
            
        self.last_face_time = self.get_clock().now()

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
            self.get_logger().info("Saludo ejecutado con éxito.")
        except Exception as e:
            self.get_logger().error(f"Fallo en el servicio de saludo: {e}")

    def state_machine_loop(self):
        now = self.get_clock().now()
        
        # Evaluar tiempo desde la última vez que se vio el rostro
        time_since_last_face = 999.0
        if self.last_face_time is not None:
            time_since_last_face = (now - self.last_face_time).nanoseconds * 1e-9

        if self.state == State.IDLE:
            # Si detectamos un rostro fresco (hace menos de 2.0 seg)
            if time_since_last_face < 2.0:
                self.state = State.APPROACHING
                self.enable_person_follower(True)
                self.get_logger().info("Persona detectada. Acercándose...")

        elif self.state == State.APPROACHING:
            # Si perdemos a la persona antes de llegar, volvemos a IDLE
            if time_since_last_face > 10.0:
                self.state = State.IDLE
                self.enable_person_follower(False)
                self.get_logger().info("Se perdió a la persona durante el acercamiento. Buscando otra...")
                return
                
            # Nos detenemos al llegar a 1 metro de la persona para saludar
            if self.current_distance <= 0.5 and time_since_last_face < 2.0:
                self.state = State.GREETING
                self.enable_person_follower(False) # Detenemos el seguidor
                self.pub_cmd_vel.publish(Twist()) # Freno total de seguridad
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
            # Si no hay rostros interactuando por 15 segundos
            if time_since_last_face > 15.0:
                self.state = State.IDLE
                self.get_logger().info("La persona se ha ido. Buscando a alguien más...")

def main(args=None):
    rclpy.init(args=args)
    node = GreetAndReturnSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()