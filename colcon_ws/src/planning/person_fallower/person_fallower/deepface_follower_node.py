#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Int32
from cv_bridge import CvBridge
import cv2
import math

try:
    from deepface import DeepFace
except ImportError:
    raise ImportError("Por favor instala deepface: pip install deepface")


class DeepFaceFollowerNode(Node):
    def __init__(self):
        super().__init__('deepface_follower_node')
        
        # Parámetros ajustables
        # El target_face_height_px representa cómo se ve un rostro promedio a ~0.5 metros
        # en tu resolución de cámara. (Requiere calibración: párate a 0.5m y revisa el print).
        self.declare_parameter("target_face_height_px", 180.0) 
        self.declare_parameter("target_distance_m", 0.5) 
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("show_debug_window", True)
        
        self.target_face_h = self.get_parameter("target_face_height_px").value
        self.target_dist = self.get_parameter("target_distance_m").value
        self.show_debug_window = self.get_parameter("show_debug_window").value
        
        self.bridge = CvBridge()
        self.is_processing = False
        
        # Subs y Pubs
        self.sub_img = self.create_subscription(
            Image, 
            self.get_parameter("image_topic").value, 
            self.image_callback, 
            10
        )
        self.pub_age = self.create_publisher(Int32, '/person_follower/age', 10)
        self.pub_distance = self.create_publisher(Float32, '/person_follower/distance', 10)
        self.pub_error_x = self.create_publisher(Float32, '/person_follower/error_x', 10)
        self.pub_error_z = self.create_publisher(Float32, '/person_follower/error_z', 10)
        
        self.get_logger().info("DeepFace Follower Iniciado. Buscando rostros...")

    def image_callback(self, msg: Image):
        # Si ya estamos procesando una imagen, ignoramos este frame (evita lag)
        if self.is_processing:
            return
            
        self.is_processing = True

        try:
            # 1. Convertir ROS Image a OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img_h, img_w, _ = cv_img.shape
            
            try:
                # 2. Extraer rostros y analizar la edad usando DeepFace
                objs = DeepFace.analyze(
                    img_path=cv_img, 
                    actions=['age'],
                    detector_backend='opencv', 
                    enforce_detection=True,
                    align=False,
                    silent=True
                )
                
                if objs:
                    # Asegurar compatibilidad en caso de que detecte múltiples personas
                    if not isinstance(objs, list):
                        objs = [objs]

                    # Quedarse con el rostro más grande detectado (posiblemente la persona más cercana)
                    largest_face = max(objs, key=lambda f: f['region']['w'] * f['region']['h'])
                    area = largest_face['region']
                    x, y, w, h = area['x'], area['y'], area['w'], area['h']
                    age = largest_face['age']
                    
                    # 3. Control Angular (Alineación YAW)
                    # El objetivo es que el rostro quede en el centro de la imagen (img_w / 2)
                    center_x = x + (w / 2.0)
                    error_x = (img_w / 2.0) - center_x 
                    
                    # 4. Control Lineal (Distancia ~ 0.5m)
                    # Si la altura actual (h) es menor al target, significa que la persona está lejos
                    error_z = self.target_face_h - h 
                    
                    # 5. Calcular Distancia y publicar
                    # Usando proporción inversa en base a la calibración de distancia
                    distance = (self.target_dist * self.target_face_h) / float(h)
                    
                    msg_age = Int32()
                    msg_age.data = int(age)
                    self.pub_age.publish(msg_age)
                    
                    msg_dist = Float32()
                    msg_dist.data = distance
                    self.pub_distance.publish(msg_dist)

                    msg_err_x = Float32()
                    msg_err_x.data = float(error_x)
                    self.pub_error_x.publish(msg_err_x)
                    
                    msg_err_z = Float32()
                    msg_err_z.data = float(error_z)
                    self.pub_error_z.publish(msg_err_z)
                    
                # Dibujar recuadro e información si el modo debug está activo
                if self.show_debug_window:
                    cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(cv_img, f"Edad: {int(age)} Dist: {distance:.2f}m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
            except ValueError:
                # DeepFace arroja ValueError cuando no detecta un rostro en la imagen
                pass 
            except Exception as e:
                self.get_logger().error(f"Error detectando rostro: {e}")

            # Mostrar la imagen de la cámara en pantalla
            if self.show_debug_window:
                cv2.imshow("DeepFace Debug Window", cv_img)
                cv2.waitKey(1)

        finally:
            # Pase lo que pase, liberar la bandera para permitir el siguiente frame
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = DeepFaceFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()