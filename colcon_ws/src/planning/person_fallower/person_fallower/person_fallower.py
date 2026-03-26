#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState

class PersonFollowerBase(Node):
    def __init__(self):
        super().__init__('person_fallower_base')
        
        # Parámetros PID y límites de velocidad
        self.declare_parameter("kp_linear", 0.002, description="Ganancia proporcional para control de distancia (basado en píxeles)")
        self.declare_parameter("kp_angular", 1.2, description="Ganancia proporcional para que el cuerpo siga a la cabeza (basado en radianes)")
        self.declare_parameter("max_linear_vel", 0.2)
        self.declare_parameter("max_angular_vel", 0.5)
        
        self.kp_lin = self.get_parameter("kp_linear").value
        self.kp_ang = self.get_parameter("kp_angular").value
        self.max_lin = self.get_parameter("max_linear_vel").value
        self.max_ang = self.get_parameter("max_angular_vel").value
        
        # Variables de estado interno
        self.head_pan_angle = 0.0
        self.error_z = 0.0
        self.enabled = False  # Por defecto apagado, la máquina de estados lo encenderá
        
        # Suscriptores
        self.sub_enable = self.create_subscription(Bool, '/person_follower/enable', self.enable_callback, 10)
        self.sub_joints = self.create_subscription(
            JointState, '/joint_states', self.joints_callback, 10
        )
        self.sub_err_z = self.create_subscription(Float32, '/person_follower/error_z', self.err_z_callback, 10)
        
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
        self.head_pan_angle = msg.position[0]

    def err_z_callback(self, msg: Float32):
        self.error_z = msg.data

    def control_loop(self):
        if not self.enabled:
            return
            
        twist = Twist()
        
        # Calcular velocidades
        # El error angular es el ángulo de la cabeza. El cuerpo debe girar para que la cabeza vuelva a 0.
        cmd_ang_z = self.clamp(self.kp_ang * self.head_pan_angle, self.max_ang)
        cmd_lin_x = self.clamp(self.kp_lin * self.error_z, self.max_lin)
        
        # Zonas muertas
        if abs(self.error_z) < 15:
            cmd_lin_x = 0.0
        # La zona muerta para el giro ahora es en radianes
        if abs(self.head_pan_angle) < 0.05: # ~3 grados
            cmd_ang_z = 0.0
            
        twist.linear.x = float(cmd_lin_x)
        twist.angular.z = float(cmd_ang_z)
        
        self.pub_cmd_vel.publish(twist)
        
        # El decaimiento angular ya no es necesario aquí. El decaimiento lineal sí.
        self.error_z *= 0.8

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerBase()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()