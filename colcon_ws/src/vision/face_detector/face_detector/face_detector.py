import sys
import os
import site

# Bloquear el acceso a las librerías globales (~/.local/...)
user_site = site.getusersitepackages()
if user_site in sys.path:
    sys.path.remove(user_site)

# Inyección del virtual environtment
venv_site_packages = '/home/booster/Pumanoids/colcon_ws/face_detector_env/lib/python3.10/site-packages'
if venv_site_packages not in sys.path:
    sys.path.insert(0, venv_site_packages)

# Imports normales
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import time
from geometry_msgs.msg import Point
from pumas_vision_msgs.msg import VisionObject

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        self.prev_time = 0.0
        
        # Subscribers
        self.subscription = self.create_subscription(
            Image, 
            'image_raw', 
            self.listener_callback, 
            10
        )
            
        # Publishers
        self.publisher_ = self.create_publisher(Image, '/face_detection/image', 10)

        self.target_pub = self.create_publisher(VisionObject, '/vision/face', 1)
        
        self.bridge = CvBridge()
        
        # Inicialización de MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(
            model_selection=0, # 0 para rostros cercanos (a menos de 2 metros)
            min_detection_confidence=0.5)

        self.prev_time = 0.0

    def get_vision_object_msg(self, id, confidence, img_x, img_y, width, height, cartesian_x, cartesian_y):
        msg = VisionObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.id = id
        msg.confidence = confidence
        msg.x = int(img_x)
        msg.y = int(img_y)
        msg.width = int(width)
        msg.height = int(height)
        msg.pose.position.x = cartesian_x
        msg.pose.position.y = cartesian_y
        msg.pose.position.z = 0.0
        
        return msg

    def listener_callback(self, msg):
        current_time = time.time()
        # Convertir el mensaje de ROS 2 a una imagen de OpenCV (BGR)
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # MediaPipe requiere que la imagen esté en formato RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Procesar la imagen para buscar rostros
        results = self.face_detection.process(rgb_frame)

        # Buscar el rostro más cercano dependiendo del área del bounding box
        if results.detections:
            rostro_mas_cercano = None
            area_maxima = 0.0

            for detection in results.detections:
                bbox = detection.location_data.relative_bounding_box
                area = bbox.width * bbox.height
                
                if area > area_maxima:
                    area_maxima = area
                    rostro_mas_cercano = detection

            if rostro_mas_cercano:
                # Coordenadas en valores relativos de 0 a 1
                h, w, c = frame.shape
                bbox = rostro_mas_cercano.location_data.relative_bounding_box
                xmin = int(bbox.xmin * w)
                ymin = int(bbox.ymin * h)
                width = int(bbox.width * w)
                height = int(bbox.height * h)
                
                # Bounding box de detección
                cv2.rectangle(frame, (xmin, ymin), (xmin + width, ymin + height), (0, 255, 0), 2)
                cv2.putText(frame, "Person", (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                x_center = xmin + (width / 2.0)
                y_center = ymin + (height / 2.0)

                confidence = rostro_mas_cercano.score[0]

                # target_msg = Point()
                # target_msg.x = float(x_center)
                # target_msg.y = float(y_center)
                # target_msg.z = 0.0

                # Calcular distancia estimada (180px representa ~0.5m)
                distancia = (0.5 * 180.0) / height if height > 0 else 0.0

                cv2.putText(frame, f"Dist: {distancia:.2f}m", (xmin, ymin - 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                vision_obj_msg = self.get_vision_object_msg(
                    "face", 
                    float(confidence), 
                    x_center, 
                    y_center, 
                    width, 
                    height, 
                    float(distancia), 
                    0.0
                )

                self.target_pub.publish(vision_obj_msg)

                cv2.circle(frame, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)

        # Cálculo de FPS
        fps = 1.0 / (current_time - self.prev_time) if self.prev_time > 0 else 0.0
        self.prev_time = current_time
        
        cv2.putText(frame, f'FPS: {int(fps)}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
        
        # Publicar la imagen
        annotated_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    face_detector = FaceDetector()
    
    try:
        rclpy.spin(face_detector)
    except KeyboardInterrupt:
        pass
    finally:
        face_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#ros2 run robot_vision face_detector --ros-args -r image_raw:=/vision/front_cam/image_color
