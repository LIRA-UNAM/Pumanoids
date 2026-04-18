
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from localization_msg.msg import VisionLandmark, VisionLandmarkArray
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
import math
import torch
import torchvision.transforms as T
import cv2
import sys
from pathlib import Path
import os
from scipy.optimize import least_squares

DETR_ROOT = os.environ.get("DETR_ROOT")
if DETR_ROOT is None:
    raise RuntimeError(
        "DETR_ROOT environment variable is not set. "
        "Please export DETR_ROOT=/path/to/Deformable-DETR"
    )
DETR_ROOT = Path(DETR_ROOT)
sys.path.insert(0, str(DETR_ROOT))
from models import build_model
from util.misc import nested_tensor_from_tensor_list
# Enum de landmarks (DEBE coincidir con tu msg)
# -----------------------
class DeformableDETRNode(Node):
    def __init__(self):
        super().__init__("detector_node")
        # ORDEN DEBE COINCIDIR con el entrenamiento del modelo
        self.class_names = [
            "ball",
            "goal",
            "robot",
            "L",
            "T",
            "X"
            # "robot"
        ]
        self.landmark_ids = {
            "ball": 0,
            "goal": 1,
            "robot": 2,
            "L": 3,
            "T": 4,
            "X": 5,
            # "robot": 6,
        }
        self.head_yaw = 0.0
        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )
        # 1: goal, 3: L, 4: T, 5: X
        self.map_landmarks = {
            3: [(-0.8, 1.3),
                (-0.8, -1.3),
                ( 0.8, 1.3),
                ( 0.8, -1.3),

                (-0.5, 0.9),
                ( 0.5,  0.9),
                ( -0.5, -0.9),
                ( 0.5, -0.9),

                ( -0.23, -1.15),
                ( 0.23, -1.15),
                ( 0.23, 1.15),
                ( -0.23, 1.15),], # L-corners
            4: [(-0.5, 1.3),
                (0.5, 1.3),
                (0.5, -1.3),
                (-0.5, -1.3),

                (-0.23,  1.3),
                (0.23, 1.3),
                (-0.23,  -1.3),
                (0.23, -1.3),

                (-0.8, 0.0),
                (0.8, 0.0),
                ], # T-junctions
            5: [(-0.2, 0.9),
                (0.2, 0.9),
                (-0.2, -0.9),
                (0.2, -0.9),
                (-0.5,  0.0),
                (0.5, 0.0),
                ] # X-cross
        }
        self.declare_parameter("checkpoint")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("visualize", True)
        self.visualize = self.get_parameter("visualize").value
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        self.bridge = CvBridge()

        self.model, self.postprocessors = self.load_model(
            self.get_parameter("checkpoint").value
        )
        topic = self.get_parameter("image_topic").value
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            qos_profile_sensor_data
    
        )
        self.obs_pub = self.create_publisher(
            VisionLandmarkArray,
            '/vision/landmarks',
            10
        )
        self.debug_img_pub = self.create_publisher(
            Image,
            '/vision/image_detections_pub',
            10
        )
        # FOV
        self.camera_fov = math.radians(53.7)
    def wrap_to_pi(self, angle): #(-pi, pi]
        return (angle + math.pi) % (2.0 * math.pi) - math.pi
    def joint_callback(self, msg):
        if "HeadYaw" in msg.name:
            idx = msg.name.index("HeadYaw")
            self.head_yaw = msg.position[idx]
    def load_model(self, checkpoint_path):
        self.get_logger().info(f"Loading checkpoint: {checkpoint_path}")
        checkpoint = torch.load(checkpoint_path, map_location="cpu")
        args = checkpoint["args"]
        model, _, postprocessors = build_model(args)
        model.load_state_dict(checkpoint["model"], strict=False)
        model.to(self.device)
        model.eval()
        return model, postprocessors
    def angular_prune(self, detections, min_sep_rad):
        dets = sorted(detections, key=lambda d: d["angle"])
        filtered = []
        for det in dets:
            # Verificar si está muy cerca de ALGUNA detección ya aceptada
            too_close = False
            for accepted in filtered:
                angle_diff = abs(self.angle_diff(det["angle"], accepted["angle"]))
                if angle_diff < min_sep_rad:
                    too_close = True
                    break
            
            if not too_close:
                filtered.append(det)
        
        return filtered
    def angle_diff(self, a, b):
        d = a - b
        return math.atan2(math.sin(d), math.cos(d))
    def image_callback(self, msg):
        detections_by_class = {}
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)

        transform = T.Compose([
            T.ToTensor(),
            T.Normalize([0.485, 0.456, 0.406],
                        [0.229, 0.224, 0.225])
        ])

        img_tensor = transform(img_rgb).unsqueeze(0).to(self.device)
        with torch.no_grad():
            outputs = self.model(img_tensor)

        h, w, _ = img_rgb.shape
        orig_size = torch.tensor([[h, w]], device=self.device)

        results = self.postprocessors["bbox"](outputs, orig_size)[0]

        scores = results["scores"]
        labels = results["labels"]

        keep = scores > 0.58

        boxes  = results["boxes"][keep]
        labels = labels[keep]
        scores = scores[keep]

        msg_out = VisionLandmarkArray()
        for box, label, score in zip(boxes, labels, scores):
            class_id = int(label.item())
            # 1. Validar que el ID esté dentro del rango de nuestra lista
            if class_id >= len(self.class_names):
                self.get_logger().warn(f"Detectado class_id {class_id} fuera de rango. Saltando...")
                # Debug temporal
                print(f"DEBUG: detectado ID {class_id} | len(class_names) es {len(self.class_names)}")
                continue
            class_name = self.class_names[class_id]
            # avoid critic error 
            if class_name not in self.landmark_ids:
                continue
            #--------angle estimation--------
                # get tensor to a standard int list and unpacking 
            x0, y0, x1, y1 = box.int().tolist()
            x_center = 0.5 * (x0 + x1)
            # eje optico
            img_center_x = w / 2.0
            # -1 to +1 left to right
            norm_x = (x_center - img_center_x) / img_center_x
            # left robot-positive
            alpha_cam = -(norm_x * (self.camera_fov / 2.0))
            
            if abs(alpha_cam) > (self.camera_fov / 2.0):
                continue
            alpha_body = self.wrap_to_pi(alpha_cam + self.head_yaw) # angle with pan correction
            det = {
                "class_name": class_name,
                "class_id": self.landmark_ids[class_name],
                "x_center": x_center,
                "angle": float(alpha_body),
                "confidence": float(score),
                "box": (x0, y0, x1, y1),
            }
            detections_by_class.setdefault(class_name, []).append(det)
        msg_out = VisionLandmarkArray()
        
        MIN_ANG_SEP = math.radians(9.0) 
        for class_name, dets in detections_by_class.items():
            pruned = self.angular_prune(dets, MIN_ANG_SEP)

            for det in pruned:
                lm = VisionLandmark()
                lm.id = det["class_id"]
                lm.angle = det["angle"]
                lm.confidence = det["confidence"]
                msg_out.landmarks.append(lm)
                x0, y0, x1, y1 = det["box"]
                text = f"{class_name} {det['confidence']:.2f}"
 
                cv2.rectangle(cv_img, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(
                    cv_img,
                    text,
                    (x0, max(y0 - 5, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
        if msg_out.landmarks:
            self.obs_pub.publish(msg_out)
        try:
            # Convertir de nuevo a mensaje ROS
            debug_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
            debug_msg.header = msg.header # Mantener el timestamp original
            self.debug_img_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing debug image: {e}")
        if self.visualize:
            cv2.imshow("Def-DETR detections", cv_img)
            cv2.waitKey(1)
def main():
    rclpy.init()
    node = DeformableDETRNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
