#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float32MultiArray
from pumas_vision_msgs.msg import VisionObject
from cv_bridge import CvBridge
import cv2
import math

try:
    from deepface import DeepFace
except ImportError:
    raise ImportError("Por favor instala deepface: pip install deepface")

HFOV = (86 * math.pi) / 180.0

class DeepFaceFollowerNode(Node):
    def __init__(self):
        super().__init__('deepface_follower_node')
        
        # Parámetros ajustables
        # El target_face_height_px representa cómo se ve un rostro promedio a ~0.5 metros
        # en tu resolución de cámara. (Requiere calibración: párate a 0.5m y revisa el print).
        self.declare_parameter("target_face_height_px", 90.0) 
        self.declare_parameter("target_distance_m", 0.5) 
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("show_debug_window", True)
        
        self.target_face_h = self.get_parameter("target_face_height_px").value
        self.target_dist = self.get_parameter("target_distance_m").value
        self.show_debug_window = self.get_parameter("show_debug_window").value
        
        self.bridge = CvBridge()
        self.is_processing = False
        
        self.current_pan = 0.0
        self.current_tilt = 0.0
        self.goal_pan = 0.0
        self.goal_tilt = 0.0
        
        # Variables para el escaneo de búsqueda
        self.last_face_time = self.get_clock().now()
        self.last_scan_time = self.get_clock().now()
        self.look_for_poses = [[0.0, 0.0], [-0.8, 0.0], [0.8, 0.0], [-0.8, 0.3], [0.8, 0.3], [0.0, 0.3]]
        self.pose_index = 0
        
        # Subs y Pubs
        self.sub_img = self.create_subscription(
            Image, 
            self.get_parameter("image_topic").value, 
            self.image_callback, 
            1
        )
        self.sub_joints = self.create_subscription(
            JointState, 
            '/joint_states', 
            self.callback_joints, 
            10
        )
        self.pub_face = self.create_publisher(VisionObject, '/vision/face', 10)
        self.pub_head = self.create_publisher(Float32MultiArray, '/hardware/head/goal_pose', 10)
        
        self.get_logger().info("DeepFace Follower Iniciado. Buscando rostros...")

    def callback_joints(self, msg: JointState):
        if len(msg.position) >= 2:
            self.current_pan = msg.position[0]
            self.current_tilt = msg.position[1]

    def image_callback(self, msg: Image):
        # Si ya estamos procesando una imagen, ignoramos este frame (evita lag)
        if self.is_processing:
            return
            
        self.is_processing = True

        try:
            # 1. Convertir ROS Image a OpenCV
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Reducir resolución fuertemente a 320x240 para acelerar el procesamiento
            cv_img = cv2.resize(cv_img, (320, 240))
            img_h, img_w, _ = cv_img.shape

            try:
                # 2. Extraer rostros directamente (bypassea modelos pesados como edad/emociones)
                faces = DeepFace.extract_faces(
                    img_path=cv_img, 
                    detector_backend='opencv', 
                    enforce_detection=True,
                    align=False
                )
                
                if faces:
                    # Quedarse con el rostro más grande detectado
                    largest_face = max(faces, key=lambda f: f['facial_area']['w'] * f['facial_area']['h'])
                    area = largest_face['facial_area']
                    x, y, w, h = area['x'], area['y'], area['w'], area['h']
                    
                    self.last_face_time = self.get_clock().now()
                    
                    # 3. Control Angular (Alineación YAW)
                    # El objetivo es que el rostro quede en el centro de la imagen (img_w / 2)
                    center_x = x + (w / 2.0)
                    center_y = y + (h / 2.0)
                    
                    # 4. Calcular Distancia Real y Proyección Cartesiana 
                    # Usando proporción inversa en base a la calibración de distancia
                    distance = (self.target_dist * self.target_face_h) / float(h)
                    
                    theta = -(center_x - img_w / 2.0) * HFOV / img_w + self.current_pan
                    face_x = distance * math.cos(theta)
                    face_y = distance * math.sin(theta)
                    
                    # 5. Publicar como VisionObject 
                    vision_msg = VisionObject()
                    vision_msg.header.stamp = self.get_clock().now().to_msg()
                    vision_msg.header.frame_id = "camera_color_optical_frame"
                    vision_msg.id = "face"
                    vision_msg.confidence = 1.0
                    vision_msg.x = int(center_x)
                    vision_msg.y = int(center_y)
                    vision_msg.width = int(w)
                    vision_msg.height = int(h)
                    vision_msg.pose.position.x = float(face_x) # Distancia frontal
                    vision_msg.pose.position.y = float(face_y) # Desplazamiento lateral
                    vision_msg.pose.position.z = 0.0
                    self.pub_face.publish(vision_msg)
                    
                    # 6. Control Dinámico de Cabeza (Pan y Tilt)
                    # Normalizamos el error (entre -1 y 1)
                    error_pan = -(center_x - (img_w / 2.0)) / (img_w / 2.0)
                    error_tilt = (center_y - (img_h / 2.0)) / (img_h / 2.0)
                    
                    # Incrementamos suavemente la posición deseada de la cabeza
                    self.goal_pan += 0.15 * error_pan
                    self.goal_tilt += 0.15 * error_tilt
                    
                    # Limitamos los ángulos (el tilt negativo permite mirar hacia arriba)
                    self.goal_pan = max(-1.0, min(1.0, self.goal_pan))
                    self.goal_tilt = max(-0.6, min(0.8, self.goal_tilt))
                    
                    msg_head = Float32MultiArray()
                    msg_head.data = [float(self.goal_pan), float(self.goal_tilt)]
                    self.pub_head.publish(msg_head)
                    
                    # Dibujar recuadro e información si el modo debug está activo
                    if self.show_debug_window:
                        cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(cv_img, f"Dist: {distance:.2f}m", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    
            except ValueError:
                # DeepFace lanza ValueError si no encuentra rostros (enforce_detection=True)
                now = self.get_clock().now()
                time_since_last = (now - self.last_face_time).nanoseconds * 1e-9
                
                if time_since_last > 2.0:
                    # Activar modo escaneo si lleva 2 segundos sin ver a nadie
                    if (now - self.last_scan_time).nanoseconds * 1e-9 > 1.5:
                        pose = self.look_for_poses[self.pose_index]
                        self.pose_index = (self.pose_index + 1) % len(self.look_for_poses)
                        self.goal_pan = pose[0]
                        self.goal_tilt = pose[1]
                        
                        msg_head = Float32MultiArray()
                        msg_head.data = [float(self.goal_pan), float(self.goal_tilt)]
                        self.pub_head.publish(msg_head)
                        self.last_scan_time = now
                else:
                    # Perdimos el rostro hace muy poco, regresar al centro lentamente por si vuelve
                    self.goal_pan *= 0.95
                    self.goal_tilt *= 0.95
                    msg_head = Float32MultiArray()
                    msg_head.data = [float(self.goal_pan), float(self.goal_tilt)]
                    self.pub_head.publish(msg_head)
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