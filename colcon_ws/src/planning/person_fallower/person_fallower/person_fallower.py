#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PersonFollowerBase(Node):
    def __init__(self):
        super().__init__('person_fallower_base')
        
        # Parámetros PID y límites de velocidad
        self.declare_parameter("kp_linear", 0.002)
        self.declare_parameter("kp_angular", 0.003)
        self.declare_parameter("max_linear_vel", 0.2)
        self.declare_parameter("max_angular_vel", 0.5)
        
        self.kp_lin = self.get_parameter("kp_linear").value
        self.kp_ang = self.get_parameter("kp_angular").value
        self.max_lin = self.get_parameter("max_linear_vel").value
        self.max_ang = self.get_parameter("max_angular_vel").value
        
        # Variables de estado interno
        self.error_x = 0.0
        self.error_z = 0.0
        
        # Suscriptores a los tópicos de error del detector
        self.sub_err_x = self.create_subscription(Float32, '/person_follower/error_x', self.err_x_callback, 10)
        self.sub_err_z = self.create_subscription(Float32, '/person_follower/error_z', self.err_z_callback, 10)
        
        # Publicador de Twist
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Bucle de control constante a 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Controlador de Base (Twist) Iniciado.")

    def clamp(self, value, max_value):
        return max(-max_value, min(max_value, value))

    def err_x_callback(self, msg: Float32):
        self.error_x = msg.data

    def err_z_callback(self, msg: Float32):
        self.error_z = msg.data

    def control_loop(self):
        twist = Twist()
        
        # Calcular velocidades
        cmd_lin_x = self.clamp(self.kp_lin * self.error_z, self.max_lin)
        cmd_ang_z = self.clamp(self.kp_ang * self.error_x, self.max_ang)
        
        # Zonas muertas
        if abs(self.error_z) < 15:
            cmd_lin_x = 0.0
        if abs(self.error_x) < 20:
            cmd_ang_z = 0.0
            
        twist.linear.x = float(cmd_lin_x)
        twist.angular.z = float(cmd_ang_z)
        
        self.pub_cmd_vel.publish(twist)
        
        # Decaimiento del error (Detiene al robot suavemente si DeepFace pierde el rostro)
        self.error_x *= 0.8
        self.error_z *= 0.8

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()