import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image, JointState, CameraInfo
from cv_bridge import CvBridge
from pumas_vision_msgs.msg import VisionObject
import numpy
import cv2
import os
from ultralytics import YOLO

ball_radious = 0.11
head_x = 0.0
head_y = 0.0
head_z = 1.105


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

    def callback_camera_info(self, msg):
        if not self.has_camera_info:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.has_camera_info = True

    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(img_bgr, verbose=False)

        idxs = results[0].boxes.cls.cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()
        bboxes = results[0].boxes.xywh.cpu().tolist()

        for i in range(len(idxs)):
            name = results[0].names[idxs[i]]
            if "ball" in name:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                ball_x, ball_y = self.get_ball_position(x_center, y_center, msg.width, msg.height)
                vision_obj_msg = self.get_vision_object_msg(name, float(confidence), x_center, y_center, width, height, ball_x, ball_y)
                self.pub_ball.publish(vision_obj_msg)
            elif "goalpost" in name:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                vision_obj_msg = self.get_vision_object_msg(name, float(confidence), x_center, y_center, width, height, 0.0, 0.0)
                self.pub_goalpost.publish(vision_obj_msg)

        annotated_frame = results[0].plot()
        cv2.imshow("YOLO Detection", annotated_frame)
        cv2.waitKey(1)

    def get_ball_position(self, image_x, image_y, img_width, img_height):
        if not self.has_camera_info:
            return 0.0, 0.0

        u = image_x
        v = image_y

        psi = numpy.arctan2((u - self.cx), self.fx)
        phi = numpy.arctan2((v - self.cy), self.fy)

        total_pan = self.current_head_pan + psi
        total_tilt = self.current_head_tilt + phi

        if abs(numpy.tan(total_tilt)) < 1e-3:
            return 0.0, 0.0

        R = (head_z - ball_radious) / numpy.tan(total_tilt)

        ball_x = head_x + R * numpy.cos(total_pan)
        ball_y = head_y + R * numpy.sin(total_pan)

        return float(ball_x), float(ball_y)

    def __init__(self):
        print("INITIALIZING BALL DETECTOR NODE - ")
        super().__init__("ball_detector")
        self.current_head_pan = 0.0
        self.current_head_tilt = 0.0
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.has_camera_info = False
        self.br = CvBridge()
        model_path = os.path.join(get_package_share_directory("ball_detector"), "models", "ball_model.pt")
        self.declare_parameter('model_path', model_path)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        self.pub_ball = self.create_publisher(VisionObject, '/vision/ball', 1)
        self.pub_goalpost = self.create_publisher(VisionObject, '/vision/goalpost', 1)
        self.sub_joints = self.create_subscription(JointState, "/joint_states", self.callback_joint_states, 1)
        self.sub_cam_info = self.create_subscription(CameraInfo, "/camera/color/camera_info", self.callback_camera_info, 1)


def main(args=None):
    rclpy.init(args=args)
    ball_detector_node = BallDetectorNode()
    rclpy.spin(ball_detector_node)
    ball_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
