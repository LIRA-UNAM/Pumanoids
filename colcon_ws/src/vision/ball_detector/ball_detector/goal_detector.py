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
POST_HEIGHT = 0.56
FOCAL_LENGTH = 580
head_x = 0.0
head_y = 0.0


class GoalDetectorNode(Node):
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

    def callback_goal_robot_pose(self, msg):
        self.latest_goal_robot_pose = msg

    def callback_img(self, msg):
        img_bgr = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(img_bgr, verbose=False)

        idxs = results[0].boxes.cls.cpu().tolist()
        confs = results[0].boxes.conf.cpu().tolist()
        bboxes = results[0].boxes.xywh.cpu().tolist()

        detected_goalposts = []
        goal_center_found = False
        avg_img_x = 0.0
        avg_img_y = 0.0

        for i in range(len(idxs)):
            name = results[0].names[idxs[i]]
            if "goal" in name:
                confidence = confs[i]
                x_center, y_center, width, height = bboxes[i]
                post_x, post_y = self.get_goalpost_position(x_center, height, msg.width)

                img_base_y = y_center + (height / 2.0)

                detected_goalposts.append({
                    'x': post_x,
                    'y': post_y,
                    'confidence': confidence,
                    'img_x': x_center,
                    'img_y': img_base_y,
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
            goal_center_found = True
            avg_width = (p1['width'] + p2['width']) / 2.0
            avg_height = (p1['height'] + p2['height']) / 2.0
            avg_confidence = (p1['confidence'] + p2['confidence']) / 2.0

            vision_obj_msg = self.get_vision_object_msg(
                "goal_center", float(avg_confidence), avg_img_x, avg_img_y,
                avg_width, avg_height, goal_center_x, goal_center_y
            )
            self.pub_goal.publish(vision_obj_msg)

        elif len(detected_goalposts) == 1:
            p = detected_goalposts[0]
            vision_obj_msg = self.get_vision_object_msg(
                "goalpost", float(p['confidence']), p['img_x'], p['img_y'],
                p['width'], p['height'], p['x'], p['y']
            )
            self.pub_goal.publish(vision_obj_msg)

        if self.get_parameter('show_debug_window').get_parameter_value().bool_value:
            annotated_frame = results[0].plot()
            if goal_center_found:
                cv2.circle(annotated_frame, (int(avg_img_x), int(avg_img_y)), 10, (0, 255, 0), -1)
                cv2.putText(
                    annotated_frame, "GOAL CENTER", (int(avg_img_x) - 40, int(avg_img_y) - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )
            if self.latest_goal_robot_pose is not None:
                gx, gy = self.latest_goal_robot_pose.x, self.latest_goal_robot_pose.y
                cv2.circle(annotated_frame, (gx, gy), 10, (255, 165, 0), -1)
                cv2.putText(
                    annotated_frame, "GOAL ROBOT POSE", (gx - 50, gy - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2
                )
            cv2.imshow("Goal Detection", annotated_frame)
            cv2.waitKey(1)

    def get_goalpost_position(self, img_x, img_h, img_width):
        if img_h < 1:
            return 0.0, 0.0

        distance = (FOCAL_LENGTH * POST_HEIGHT) / img_h
        theta = -(img_x - img_width / 2) * HFOV / img_width + self.current_head_pan
        post_x = head_x + distance * numpy.cos(theta)
        post_y = head_y + distance * numpy.sin(theta)
        return post_x, post_y

    def __init__(self):
        print("INITIALIZING GOAL DETECTOR NODE")
        super().__init__("goal_detector")
        self.current_head_pan = 0.0
        self.current_head_tilt = 0.0
        self.br = CvBridge()
        model_path = os.path.join(
            get_package_share_directory("ball_detector"), "models", "yolov8_center.pt"
        )
        self.declare_parameter('model_path', model_path)
        self.declare_parameter('show_debug_window', False)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.sub_img = self.create_subscription(Image, '/camera/color/image_raw', self.callback_img, 1)
        self.sub_joints = self.create_subscription(JointState, '/joint_states', self.callback_joint_states, 1)
        self.sub_goal_robot_pose = self.create_subscription(
            VisionObject, '/goal_robot_pose', self.callback_goal_robot_pose, 1
        )
        self.pub_goal = self.create_publisher(VisionObject, '/vision/goal_center', 1)
        self.model = YOLO(model_path)
        self.latest_goal_robot_pose = None


def main(args=None):
    rclpy.init(args=args)
    goal_detector_node = GoalDetectorNode()
    rclpy.spin(goal_detector_node)
    goal_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
