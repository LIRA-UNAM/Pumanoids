import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import cv2
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from pumas_vision_msgs.msg import VisionObject
import numpy
import os
from ultralytics import YOLO

HFOV = (86 * 3.14159265358979323846) / 180.0
VFOV = (57 * 3.14159265358979323846) / 180.0
ball_radious = 0.11
head_x = 0.0
head_y = 0.0
head_z = 1.05


class BallDetectorNode(Node):
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
    
    def callback_joint_states(self, msg):
        self.current_head_pan = msg.position[0]
        self.current_head_tilt = msg.position[1]
    
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(img_bgr, verbose=False)

        idxs = results[0].boxes.cls.cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()
        bboxes = results[0].boxes.xywh.cpu().tolist()

        for i in range(len(idxs)):
            name = results[0].names[idxs[i]]
            if "ball" in name and confs[i] > 0.60:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                ball_x, ball_y = self.get_ball_position(x_center, y_center, msg.width, msg.height)
                vision_obj_msg = self.get_vision_object_msg(
                    name, float(confidence), x_center, y_center, width, height, ball_x, ball_y
                )
                self.pub_ball.publish(vision_obj_msg)

        if self.get_parameter('show_debug_window').get_parameter_value().bool_value:
            annotated_frame = results[0].plot()
            cv2.imshow("YOLO Detection", annotated_frame)
            cv2.waitKey(1)
        

    
    def get_ball_position(self, img_x, img_y, img_width, img_height):
        ball_x = 0.0
        ball_y = 0.0
        theta = -(img_x - img_width / 2) * HFOV / img_width + self.current_head_pan
        phi = (img_y - img_height / 2) * VFOV / img_height + self.current_head_tilt
        ux = numpy.cos(theta) * numpy.cos(phi)
        uy = numpy.sin(theta) * numpy.cos(phi)
        uz = - numpy.sin(phi)
        
        if abs(uz) < 1e-6:
            return 0.0, 0.0
        
        lambda_val = (ball_radious - head_z) / uz
        ball_x = head_x + lambda_val * ux
        ball_y = head_y + lambda_val * uy
        return ball_x, ball_y
    
    def __init__(self):
        print("INITIALIZING BALL DETECTOR NODE")
        super().__init__("ball_detector")
        self.current_head_pan = 0.0
        self.current_head_tilt = 0.0
        self.br = CvBridge()
        model_path = os.path.join(get_package_share_directory("ball_detector"), "models", "yolov8_center.pt")
        self.declare_parameter('model_path', model_path)
        self.declare_parameter('show_debug_window', False)
        self.show_debug = self.get_parameter('show_debug_window').get_parameter_value().bool_value
        model_path  = self.get_parameter('model_path').get_parameter_value().string_value
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 1)
        self.pub_ball = self.create_publisher(VisionObject, '/vision/ball', 1)
        self.model = YOLO(model_path)

def main(args=None):
    rclpy.init(args=args)
    ball_detector_node = BallDetectorNode()
    rclpy.spin(ball_detector_node)
    ball_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
