import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from localization_msg.msg import VisionLandmark, VisionLandmarkArray
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import math
import cv2
from ultralytics import YOLO

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        
        # Parámetros
        self.declare_parameter("checkpoint", "/home/booster/Pumanoids/colcon_ws/src/vision/ball_detector/models/yolov8_center.pt")
        self.declare_parameter("visualize", False)
        self.visualize = self.get_parameter("visualize").value
        
        model_path = self.get_parameter("checkpoint").value
        self.get_logger().info(f"Cargando modelo YOLO: {model_path}")
        self.model = YOLO(model_path)
        
        # Mapeo de IDs
        self.class_names = ["ball", "goal", "robot", "L", "T", "X", "center"]
        
        self.head_yaw = 0.0
        self.bridge = CvBridge()
        self.camera_fov = math.radians(93)
        
        # Suscriptores
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.joint_callback, 10)
        self.subscription = self.create_subscription(Image, "/camera/color/image_raw", self.image_callback, qos_profile_sensor_data)
        
        # Publicadores
        self.obs_pub = self.create_publisher(VisionLandmarkArray, '/vision/landmarks', 10)
        # Nuevo publicador para ver las detecciones en ROS
        self.image_pub = self.create_publisher(Image, '/vision/detections_image', 10)

    def joint_callback(self, msg):
        if "HeadYaw" in msg.name:
            idx = msg.name.index("HeadYaw")
            self.head_yaw = msg.position[idx]

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def image_callback(self, msg):
        # Convertir imagen de ROS a OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w, _ = cv_img.shape

        # Inferencia
        results = self.model.predict(cv_img, conf=0.5, verbose=False)
        
        msg_out = VisionLandmarkArray()
        detections_by_class = {}

        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].cpu().numpy() 
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                
                class_name = self.class_names[cls] if cls < len(self.class_names) else "unknown"

                # Cálculo del ángulo
                x_center = (b[0] + b[2]) / 2.0
                img_center_x = w / 2.0
                norm_x = (x_center - img_center_x) / img_center_x
                alpha_cam = -(norm_x * (self.camera_fov / 2.0))
                alpha_body = self.wrap_to_pi(alpha_cam + self.head_yaw)

                det = {
                    "class_name": class_name,
                    "class_id": cls,
                    "angle": alpha_body,
                    "confidence": conf,
                    "box": b.astype(int)
                }
                detections_by_class.setdefault(class_name, []).append(det)

        # Filtrado y Dibujado
        MIN_ANG_SEP = math.radians(9.0)
        for class_name, dets in detections_by_class.items():
            pruned = self.angular_prune(dets, MIN_ANG_SEP)
            for det in pruned:
                # Llenar mensaje de ROS
                lm = VisionLandmark()
                lm.id = det["class_id"]
                lm.angle = det["angle"]
                lm.confidence = det["confidence"]
                msg_out.landmarks.append(lm)
                
                # Dibujar en la imagen
                x0, y0, x1, y1 = det["box"]
                # Rectángulo y texto
                cv2.rectangle(cv_img, (x0, y0), (x1, y1), (0, 255, 0), 2)
                label = f"{class_name} {det['confidence']:.2f}"
                cv2.putText(cv_img, label, (x0, max(y0-10, 20)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publicar coordenadas para el MCL
        if msg_out.landmarks:
            self.obs_pub.publish(msg_out)

        # Publicar la imagen procesada a ROS (Útil para RViz2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8"))

        # Ventana local (solo si tienes pantalla conectada)
        if self.visualize:
            cv2.imshow("Yolov8 detections", cv_img)
            cv2.waitKey(1)

    def angular_prune(self, detections, min_sep_rad):
        if not detections: return []
        dets = sorted(detections, key=lambda d: d["angle"])
        filtered = [dets[0]]
        for i in range(1, len(dets)):
            d = dets[i]
            last = filtered[-1]
            diff = math.atan2(math.sin(d["angle"] - last["angle"]), math.cos(d["angle"] - last["angle"]))
            if abs(diff) > min_sep_rad:
                filtered.append(d)
            elif d["confidence"] > last["confidence"]:
                filtered[-1] = d
        return filtered

def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
