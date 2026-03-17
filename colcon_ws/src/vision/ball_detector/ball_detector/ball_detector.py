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

HFOV = (87 * 3.14159265358979323846) / 180.0
VFOV = (58 * 3.14159265358979323846) / 180.0
ball_radious = 0.07
head_x = 0.0
head_y = 0.0
head_z = 1.25
POST_HEIGHT = 0.56
FOCAL_LENGTH = 600


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
        
        detected_goalposts = []
        
        for i in range(len(idxs)):
            name=results[0].names[idxs[i]]
            if "ball" in name:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                ball_x, ball_y = self.get_ball_position(x_center, y_center, msg.width, msg.height)
                vision_obj_msg = self.get_vision_object_msg(name, float(confidence), x_center, y_center, width, height, ball_x, ball_y)
                self.pub_ball.publish(vision_obj_msg)
            elif "goalpost" in name:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                post_x, post_y = self.get_goalpost_position(x_center, height, msg.width)
                
                detected_goalposts.append({
                    'x': post_x,
                    'y': post_y,
                    'confidence': confidence,
                    'img_x': x_center,
                    'img_y': y_center,
                    'width': width,
                    'height': height
                })
                
        if len(detected_goalposts) >= 2:
                detected_goalposts.sort(key=lambda p: p['confidence'], reverse=True)
                p1 = detected_goalposts[0]
                p2 = detected_goalposts[1]
                    
                goal_center_x = (p1['x'] + p2['x']) / 2.0
                goal_center_y = (p1['y'] + p2['y']) / 2.0
                    
                avg_img_x = (p1['img_x'] + p2['img_x']) / 2.0
                avg_img_y = (p1['img_y'] + p2['img_y']) / 2.0
                avg_width = (p1['width'] + p2['width']) / 2.0
                avg_height = (p1['height'] + p2['height']) / 2.0
                avg_confidence = (p1['confidence'] + p2['confidence']) / 2.0
                
                vision_obj_msg = self.get_vision_object_msg("goal_center", float(avg_confidence), avg_img_x, avg_img_y, avg_width, avg_height, goal_center_x, goal_center_y)
                self.pub_ball.publish(vision_obj_msg)
        elif len(detected_goalposts) == 1:
                p = detected_goalposts[0]
                vision_obj_msg = self.get_vision_object_msg("goalpost", float(p['confidence']), p['img_x'], p['img_y'], p['width'], p['height'], p['x'], p['y'])
                self.pub_goal.publish(vision_obj_msg)
        

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
    
    def get_goalpost_position(self, img_x, img_h, img_width):
        if img_h < 1:
            return 0.0, 0.0
        
        distance = (FOCAL_LENGTH * POST_HEIGHT) / img_h
        theta = -(img_x - img_width / 2) * HFOV / img_width + self.current_head_pan
        post_x = head_x + distance * numpy.cos(theta)
        post_y = head_y + distance * numpy.sin(theta)
        return post_x, post_y
        
        

    def __init__(self):
        print("INITIALIZING BALL DETECTOR NODE - ")
        super().__init__("ball_detector")
        self.current_head_pan = 0.0
        self.current_head_tilt = 0.0
        self.br = CvBridge()
        model_path = os.path.join(get_package_share_directory("ball_detector"), "models", "yolov8_center.pt")
        self.declare_parameter('model_path', model_path)
        model_path  = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Iniciando modelo con:{model_path}')
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 1)
        self.pub_ball = self.create_publisher(VisionObject, '/vision/ball', 1)
        self.pub_goal = self.create_publisher(VisionObject, '/vision/goal_center', 1)
        self.model = YOLO(model_path)

def main(args=None):
    rclpy.init(args=args)
    ball_detector_node = BallDetectorNode()
    rclpy.spin(ball_detector_node)
    ball_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
