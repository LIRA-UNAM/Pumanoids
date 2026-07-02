#!/usr/bin/env python3

import math
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from carry_ball_to_goal_interfaces.srv import GetGoalRobotPose


def load_map_goal(map_yaml: str, attack_goal: str) -> tuple[str, float, float]:
    path = Path(map_yaml)
    if not path.is_file():
        raise FileNotFoundError(f"Map YAML not found: {map_yaml}")
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    frame_id = data.get("frame_id", "map")
    goals = data.get("goals", {})
    if attack_goal not in goals:
        raise ValueError(
            f"attack_goal '{attack_goal}' not in map; keys: {list(goals.keys())}"
        )
    center = goals[attack_goal]["center"]
    return frame_id, float(center["x"]), float(center["y"])


class GoalRobotPoseServiceNode(Node):
    """
    Servicio GetGoalRobotPose: pose en mapa detrás del balón, mirando la portería
    fija del YAML. Sin publisher; request vacío; response Pose2D + success.
    """

    def __init__(self):
        super().__init__("goal_robot_pose_service")
        self.enable = False
        self.declare_parameter("distance_from_ball_m", 1.25)
        #self.declare_parameter("service_name", "get_goal_robot_pose")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("attack_goal", "positive_y")
        self.declare_parameter("ball_pose_topic", "/vision/ball_kal")

        self.distance_from_ball = self.get_parameter("distance_from_ball_m").value
        #srv_name = self.get_parameter("service_name").value
        map_yaml = self.get_parameter("map_yaml").value
        attack_goal = self.get_parameter("attack_goal").value
        ball_topic = self.get_parameter("ball_pose_topic").value

        if not map_yaml:
            raise RuntimeError("map_yaml parameter is required")

        self.map_frame, self.goal_x, self.goal_y = load_map_goal(map_yaml, attack_goal)
        self.ball_pose: Optional[Pose2D] = None
        self.pub_pose = self.create_publisher(Pose2D, '/carry_ball_to_goal/point', 1)
        self.sub_ball = self.create_subscription(
            Pose2D, ball_topic, self._callback_ball, 10
        )

        self.timer = self.create_timer(0.1,self._handle_get_pose)
        #self.srv = self.create_service(GetGoalRobotPose, srv_name, self._handle_get_pose)

        # self.get_logger().info(
        #     f"GetGoalRobotPose en '{srv_name}': portería=({self.goal_x:.2f}, {self.goal_y:.2f}), "
        #     f"attack_goal={attack_goal}, balón={ball_topic}, frame={self.map_frame}"
        # )

    def _callback_ball(self, msg: Pose2D):
        self.ball_pose = msg
        self.seeing_ball = True

    def _handle_get_pose(self): # _request, response):
        if self.ball_pose is None:
            #response.success = False
            #response.pose = Pose2D()
            return #response
            

        bx = self.ball_pose.x
        by = self.ball_pose.y
        #gx, gy = self.goal_x, self.goal_y

        dx = self.goal_x - bx #gx - bx
        dy = self.goal_y - by #gy - by
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            #response.success = False
            #response.pose = Pose2D()
            return #response

        ux = dx / dist
        uy = dy / dist

        #response.success = True
        #response.pose = Pose2D()
        #response.pose.x = bx - self.distance_from_ball * ux
        #response.pose.y = by - self.distance_from_ball * uy
        #response.pose.theta = math.atan2(dy, dx)
        pose = Pose2D()
        pose.x = bx - self.distance_from_ball * ux
        pose.y = by - self.distance_from_ball * uy
        pose.theta = math.atan2(dy, dx)
        
        print("Publishing pose")
        if ( self.seeing_ball ): self.pub_pose.publish(pose)
        self.seeing_ball = False
        self.enable = False
        #return response


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
