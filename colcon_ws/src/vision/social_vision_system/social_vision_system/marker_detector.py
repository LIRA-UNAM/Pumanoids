#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from social_vision_msgs.msg import VisionObject
except ImportError:
    pass

class MarkerDetectorNode(Node):
    def __init__(self):
        super().__init__('marker_detector_node')
        self.bridge = CvBridge()
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size_m = 0.10 # Tamaño del ArUco (10 cm)

        self.declare_parameter("target_marker_id", 7)
        self.target_marker_id = self.get_parameter("target_marker_id").value

        self.declare_parameter("show_debug_window", False)
        self.show_debug_window = self.get_parameter("show_debug_window").value

        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.sub_cam_info = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.cam_info_callback, 1)
        self.pub_marker = self.create_publisher(VisionObject, '/vision/marker', 10)
        
        self.get_logger().info("Detector de marcadores iniciado.")

    def cam_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Info de la cámara recibida.")
            self.destroy_subscription(self.sub_cam_info)

    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        if ids is not None:
            # Buscar si el ID objetivo (7) está entre los marcadores detectados
            idx = np.where(ids == self.target_marker_id)[0]
            if len(idx) > 0:
                i = idx[0]
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size_m, self.camera_matrix, self.dist_coeffs)
                
                marker_corners = corners[i][0]
                center_x = np.mean(marker_corners[:, 0])
                distance = np.linalg.norm(tvecs[i][0])

                vision_msg = VisionObject()
                vision_msg.name = f"aruco_id_{self.target_marker_id}"
                vision_msg.x = float(center_x)
                vision_msg.pose.position.x = float(distance)
                self.pub_marker.publish(vision_msg)
        
        if self.show_debug_window:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            cv2.imshow("Marker Detector Debug", cv_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MarkerDetectorNode())
    rclpy.shutdown()