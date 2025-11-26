import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pumas_vision_msgs.msg import VisionObject
import numpy
import cv2
import os
from ultralytics import YOLO


class BallDetectorNode(Node):
    def get_vision_object_msg(self, id, confidence, x, y, width, height):
        msg = VisionObject()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.id = id
        msg.confidence = confidence
        msg.x = int(x)
        msg.y = int(y)
        msg.width = int(width)
        msg.height = int(height)
        
        return msg
    
    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(img_bgr, verbose=False)

        idxs = results[0].boxes.cls.cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()
        bboxes = results[0].boxes.xywh.cpu().tolist()
        
        for i in range(len(idxs)):
            name=results[0].names[idxs[i]]
            if "ball" in name:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                vision_obj_msg = self.get_vision_object_msg(name, float(confidence), x_center, y_center, width, height)
                self.pub_ball.publish(vision_obj_msg)
                break

        annotated_frame = results[0].plot()
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)

    def __init__(self):
        print("INITIALIZING BALL DETECTOR NODE - ")
        super().__init__("ball_detector")
        self.br = CvBridge()
        model_path = os.path.join(get_package_share_directory("ball_detector"), "models", "ball_model.pt")
        self.declare_parameter('model_path', model_path)
        model_path  = self.get_parameter('model_path').get_parameter_value().string_value
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
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
