#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from pumas_vision_msgs.msg import VisionObject
from carry_ball_to_goal_interfaces.srv import GetGoalRobotPose


def yaw_to_quaternion(yaw: float) -> Quaternion:
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def zero_pose() -> Pose:
    p = Pose()
    p.position.x = 0.0
    p.position.y = 0.0
    p.position.z = 0.0
    p.orientation.x = 0.0
    p.orientation.y = 0.0
    p.orientation.z = 0.0
    p.orientation.w = 1.0
    return p


class GoalRobotPoseServiceNode(Node):
    """
    Servicio GetGoalRobotPose: pose en base_link detrás del balón (misma lógica que
    goal_robot_pose) y orientación = ángulo del vector balón -> goal_center en el plano.
    """

    def __init__(self):
        super().__init__("goal_robot_pose_service")

        self.declare_parameter("distance_from_ball_m", 0.25)
        self.declare_parameter("service_name", "get_goal_robot_pose")

        self.distance_from_ball = self.get_parameter("distance_from_ball_m").value
        srv_name = self.get_parameter("service_name").value

        self.ball_x = None
        self.ball_y = None
        self.goal_x = None
        self.goal_y = None

        self.sub_ball = self.create_subscription(
            VisionObject, "/vision/ball", self._callback_ball, 10
        )
        self.sub_goal = self.create_subscription(
            VisionObject, "/vision/goal_center", self._callback_goal, 10
        )

        self.srv = self.create_service(GetGoalRobotPose, srv_name, self._handle_get_pose)

        self.get_logger().info(f"GetGoalRobotPose en '{srv_name}'")

    def _callback_ball(self, msg: VisionObject):
        self.ball_x = msg.pose.position.x
        self.ball_y = msg.pose.position.y

    def _callback_goal(self, msg: VisionObject):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

    def _handle_get_pose(self, _request, response):
        bx, by, gx, gy = self.ball_x, self.ball_y, self.goal_x, self.goal_y
        if bx is None or by is None or gx is None or gy is None:
            response.success = False
            response.pose = zero_pose()
            return response

        dx = gx - bx
        dy = gy - by
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            response.success = False
            response.pose = zero_pose()
            return response

        ux = dx / dist
        uy = dy / dist

        point_x = bx - self.distance_from_ball * ux
        point_y = by - self.distance_from_ball * uy
        yaw = math.atan2(dy, dx)

        response.success = True
        response.pose = Pose()
        response.pose.position.x = point_x
        response.pose.position.y = point_y
        response.pose.position.z = 0.0
        response.pose.orientation = yaw_to_quaternion(yaw)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GoalRobotPoseServiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
