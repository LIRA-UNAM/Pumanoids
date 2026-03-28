#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from pumas_vision_msgs.msg import VisionObject

class PersonFollowerBase(Node):
    def __init__(self):
        super().__init__('person_fallower_base')
        
        # Parámetros PID y límites de velocidad
        self.declare_parameter("kp_linear", 0.8) # Ajustado para metros (igual que el balón)
        self.declare_parameter("kp_angular", 1.2)
        self.declare_parameter("max_linear_vel", 0.2)
        self.declare_parameter("max_angular_vel", 0.5)
        
        self.kp_lin = self.get_parameter("kp_linear").value
        self.kp_ang = self.get_parameter("kp_angular").value
        self.max_lin = self.get_parameter("max_linear_vel").value
        self.max_ang = self.get_parameter("max_angular_vel").value
        
        # Variables de estado interno
        self.head_pan_angle = 0.0
        self.distance_x = 0.5
        self.face_x_img = 160 # Centro predeterminado (320/2)
        self.enabled = False  # Por defecto apagado, la máquina de estados lo encenderá
        
        # Suscriptores
        self.sub_enable = self.create_subscription(Bool, '/person_follower/enable', self.enable_callback, 10)
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joints_callback, 10
        )
        self.sub_face = self.create_subscription(VisionObject, '/vision/face', self.face_callback, 10)
        
        # Publicador de Twist
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Bucle de control constante a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Controlador de Base (Twist) Iniciado.")

    def clamp(self, value, max_value):
        return max(-max_value, min(max_value, value))

    def enable_callback(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            # Publicar un Twist de 0 para detener el robot al deshabilitarse
            self.pub_cmd_vel.publish(Twist())

    def joints_callback(self, msg: JointState):
        # El error para el giro del cuerpo es el propio ángulo de la cabeza.
        if len(msg.position) > 0:
            self.head_pan_angle = msg.position[0]

    def face_callback(self, msg: VisionObject):
        self.face_x_img = msg.x
        self.distance_x = msg.pose.position.x

    def control_loop(self):
        if not self.enabled:
            return
            
        twist = Twist()
        
        # --- Lógica idéntica a ball_follower.py ---
        # Calculamos el error en imagen (imagen de 320px, centro en 160)
        error_img = (-self.face_x_img + 160) / 320.0
        if error_img < 0:
            error_img = -math.sqrt(-error_img)
        else:
            error_img = math.sqrt(error_img)
            
        cmd_ang_z = self.clamp(self.kp_ang * (error_img + self.head_pan_angle), self.max_ang)
        
        # Control Lineal (Avanzar)
        error_z = self.distance_x - 0.5 # 0.5 metros es la distancia objetivo
        cmd_lin_x = self.clamp(self.kp_lin * error_z, self.max_lin)
        
        # Zonas muertas
        if abs(error_z) < 0.1: 
            cmd_lin_x = 0.0
        if abs(error_img + self.head_pan_angle) < 0.05: 
            cmd_ang_z = 0.0
            
        twist.linear.x = float(cmd_lin_x)
        twist.angular.z = float(cmd_ang_z)
        
        self.pub_cmd_vel.publish(twist)
        
        self.distance_x = (self.distance_x * 0.8) + (0.5 * 0.2) # Decaimiento suave
        self.face_x_img = (self.face_x_img * 0.8) + (160.0 * 0.2) # Decaimiento suave al centro

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()