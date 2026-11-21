"""ROS 2 node translating between PumaSim generic per-robot topics and Pumanoids Booster contracts."""

from __future__ import annotations

import logging
from typing import Any

import rclpy
from booster_interface.msg import (
    BoosterApiReqMsg,
    BoosterApiRespMsg,
    LowState,
    MotorState,
    Odometer,
    RawBytesMsg,
)
from booster_msgs.msg import RpcReqMsg, RpcRespMsg
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState

from pumasim_booster_bridge.booster_protocol import (
    BoosterTranslatorCore,
    ground_truth_odom_to_localization,
    recovery_state_from_imu,
)

LOGGER = logging.getLogger(__name__)


def _try_import_kick_msg():
    try:
        from brain.msg import Kick
    except ImportError:
        return None
    return Kick


def _prefix(namespace: str, topic: str) -> str:
    clean_ns = namespace.strip().rstrip("/")
    clean_topic = topic.lstrip("/")
    return f"{clean_ns}/{clean_topic}" if clean_ns else f"/{clean_topic}"


class _RobotBridgeEndpoint:
    """Per-robot subscriptions, publishers, and protocol state attached to a ROS 2 Node."""

    def __init__(
        self,
        node: Node,
        *,
        robot_name: str,
        robot_type: str,
        booster_namespace: str,
        localization_topic: str,
        relay_rgb: bool,
        relay_depth: bool,
        relay_camera_info: bool,
        enable_kick_reference: bool,
        limits: dict[str, tuple[float, float]],
        kick_msg_cls: type | None,
    ) -> None:
        self._node = node
        self._robot_name = robot_name
        self._core = BoosterTranslatorCore(
            robot_name, robot_type=robot_type, velocity_limits=limits
        )

        self._latest_imu: Imu | None = None
        self._latest_stamp: Time | None = None
        self._last_odom_stamp_ns: int | None = None
        self._low_state_msg: LowState | None = None

        sensor_best_effort = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        # RELIABLE Depth-1 matches both RELIABLE (vision /head_pose, image_transport)
        # and BEST_EFFORT (brain latestSensorQos) subscribers per DDS QoS rules.
        booster_state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        reliable_10 = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # Publishers toward PumaSim generic commands
        self._cmd_vel_pub = node.create_publisher(
            Twist, f"/{robot_name}/cmd_vel", reliable_10
        )
        self._joint_targets_pub = node.create_publisher(
            JointState, f"/{robot_name}/joint_targets", reliable_10
        )

        # Publishers toward Pumanoids Booster demo & localization
        self._low_state_pub = node.create_publisher(
            LowState, _prefix(booster_namespace, "low_state"), booster_state_qos
        )
        self._odometer_pub = node.create_publisher(
            Odometer, _prefix(booster_namespace, "odometer_state"), booster_state_qos
        )
        self._gt_loc_pub = node.create_publisher(
            Odometer, localization_topic, reliable_10
        )
        self._head_pose_pub = node.create_publisher(
            Pose, _prefix(booster_namespace, "head_pose"), booster_state_qos
        )
        self._recovery_pub = node.create_publisher(
            RawBytesMsg,
            _prefix(booster_namespace, "fall_down_recovery_state"),
            reliable_10,
        )
        self._loco_resp_pub = node.create_publisher(
            RpcRespMsg, _prefix(booster_namespace, "LocoApiTopicResp"), reliable_10
        )
        self._api_resp_pub = node.create_publisher(
            BoosterApiRespMsg,
            _prefix(booster_namespace, "booster_rpc_resp"),
            reliable_10,
        )

        self._odom_msg = Odometer()
        self._gt_loc_msg = Odometer()
        self._head_pose_msg = Pose()
        self._recovery_msg = RawBytesMsg()
        self._twist_msg = Twist()
        self._joint_target_msg = JointState()

        # Subscriptions from PumaSim generic topics
        node.create_subscription(
            Imu, f"/{robot_name}/imu/data", self._on_imu, reliable_10
        )
        node.create_subscription(
            JointState,
            f"/{robot_name}/joint_states",
            self._on_joint_states,
            reliable_10,
        )
        node.create_subscription(
            Odometry, f"/{robot_name}/odom", self._on_odom, reliable_10
        )
        node.create_subscription(
            Odometry,
            f"/{robot_name}/ground_truth/odom",
            self._on_ground_truth_odom,
            reliable_10,
        )
        node.create_subscription(
            PoseStamped, f"/{robot_name}/head_pose", self._on_head_pose, reliable_10
        )

        # Optional RGB / Depth QoS+topic bridge when consumer cannot subscribe BEST_EFFORT.
        # CameraInfo uses SensorDataQoS in vision_node and remaps directly unless requested.
        if relay_rgb:
            self._rgb_pub = node.create_publisher(
                Image,
                _prefix(booster_namespace, "boostercamera/head/rgb"),
                booster_state_qos,
            )
            node.create_subscription(
                Image,
                f"/{robot_name}/camera/image_raw",
                self._rgb_pub.publish,
                sensor_best_effort,
            )

        if relay_depth:
            self._depth_pub = node.create_publisher(
                Image,
                _prefix(booster_namespace, "boostercamera/head/depth"),
                booster_state_qos,
            )
            node.create_subscription(
                Image,
                f"/{robot_name}/camera/depth/image_raw",
                self._depth_pub.publish,
                sensor_best_effort,
            )

        if relay_camera_info:
            self._camera_info_pub = node.create_publisher(
                CameraInfo,
                _prefix(booster_namespace, "boostercamera/head/rgb/camera_info"),
                booster_state_qos,
            )
            node.create_subscription(
                CameraInfo,
                f"/{robot_name}/camera/camera_info",
                self._camera_info_pub.publish,
                sensor_best_effort,
            )

        # Subscriptions from Booster/Pumanoids demo commands
        node.create_subscription(
            RpcReqMsg,
            _prefix(booster_namespace, "LocoApiTopicReq"),
            self._on_loco_rpc,
            reliable_10,
        )
        node.create_subscription(
            BoosterApiReqMsg,
            _prefix(booster_namespace, "booster_rpc_req"),
            self._on_api_rpc,
            reliable_10,
        )

        if enable_kick_reference and kick_msg_cls is not None:
            node.create_subscription(
                kick_msg_cls,
                _prefix(booster_namespace, "kick_ball"),
                self._on_kick_ball,
                reliable_10,
            )

    def _on_imu(self, msg: Imu) -> None:
        self._latest_imu = msg
        self._latest_stamp = msg.header.stamp
        rec = recovery_state_from_imu(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
            self._core.mode,
        )
        # RawBytesMsg.msg is sequence<uint8> in Python rclpy.
        self._recovery_msg.msg = [
            rec.state,
            rec.is_recovery_available,
            rec.current_planner_index,
        ]
        self._recovery_pub.publish(self._recovery_msg)

    def _on_joint_states(self, msg: JointState) -> None:
        self._latest_stamp = msg.header.stamp
        if self._latest_imu is None:
            return
        imu = self._latest_imu
        sample = self._core.build_low_state(
            imu_quat_xyzw=(
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            ),
            imu_gyro=(
                imu.angular_velocity.x,
                imu.angular_velocity.y,
                imu.angular_velocity.z,
            ),
            imu_acc=(
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ),
            joint_names=msg.name,
            joint_positions=msg.position,
            joint_velocities=msg.velocity,
            joint_efforts=msg.effort,
        )
        if sample is None:
            return

        if self._low_state_msg is None:
            low_msg = LowState()
            low_msg.motor_state_parallel = [
                MotorState() for _ in sample.motor_state_parallel
            ]
            low_msg.motor_state_serial = [
                MotorState() for _ in sample.motor_state_serial
            ]
            self._low_state_msg = low_msg

        low_msg = self._low_state_msg
        low_msg.imu_state.rpy = list(sample.rpy)
        low_msg.imu_state.gyro = list(sample.gyro)
        low_msg.imu_state.acc = list(sample.acc)
        for dst, src in zip(
            low_msg.motor_state_parallel, sample.motor_state_parallel, strict=True
        ):
            dst.mode = src.mode
            dst.q = src.q
            dst.dq = src.dq
            dst.ddq = src.ddq
            dst.tau_est = src.tau_est
        for dst, src in zip(
            low_msg.motor_state_serial, sample.motor_state_serial, strict=True
        ):
            dst.mode = src.mode
            dst.q = src.q
            dst.dq = src.dq
            dst.ddq = src.ddq
            dst.tau_est = src.tau_est
        self._low_state_pub.publish(low_msg)

    def _on_odom(self, msg: Odometry) -> None:
        self._latest_stamp = msg.header.stamp
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self._last_odom_stamp_ns is not None and stamp_ns < self._last_odom_stamp_ns:
            self._core.reset_state()
        self._last_odom_stamp_ns = stamp_ns

        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        sample = self._core.update_relative_odometry(
            pos.x, pos.y, ori.x, ori.y, ori.z, ori.w
        )
        self._odom_msg.x = float(sample.x)
        self._odom_msg.y = float(sample.y)
        self._odom_msg.theta = float(sample.theta)
        self._odometer_pub.publish(self._odom_msg)

    def _on_ground_truth_odom(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        sample = ground_truth_odom_to_localization(
            pos.x, pos.y, ori.x, ori.y, ori.z, ori.w
        )
        self._gt_loc_msg.x = float(sample.x)
        self._gt_loc_msg.y = float(sample.y)
        self._gt_loc_msg.theta = float(sample.theta)
        self._gt_loc_pub.publish(self._gt_loc_msg)

    def _on_head_pose(self, msg: PoseStamped) -> None:
        pos = msg.pose.position
        ori = msg.pose.orientation
        self._core.update_head_pose(
            (pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w)
        )
        self._head_pose_msg.position.x = float(pos.x)
        self._head_pose_msg.position.y = float(pos.y)
        self._head_pose_msg.position.z = float(pos.z)
        self._head_pose_msg.orientation.x = float(ori.x)
        self._head_pose_msg.orientation.y = float(ori.y)
        self._head_pose_msg.orientation.z = float(ori.z)
        self._head_pose_msg.orientation.w = float(ori.w)
        self._head_pose_pub.publish(self._head_pose_msg)

    def _emit_actions(self, result) -> None:
        if result.cmd_vel is not None:
            vx, vy, wz = result.cmd_vel
            self._twist_msg.linear.x = float(vx)
            self._twist_msg.linear.y = float(vy)
            self._twist_msg.linear.z = 0.0
            self._twist_msg.angular.x = 0.0
            self._twist_msg.angular.y = 0.0
            self._twist_msg.angular.z = float(wz)
            self._cmd_vel_pub.publish(self._twist_msg)
        if result.joint_targets is not None:
            names, positions = result.joint_targets
            if self._latest_stamp is not None:
                self._joint_target_msg.header.stamp = self._latest_stamp
            self._joint_target_msg.name = list(names)
            self._joint_target_msg.position = [float(v) for v in positions]
            self._joint_targets_pub.publish(self._joint_target_msg)
        if result.reset_odometry:
            self._odom_msg.x = 0.0
            self._odom_msg.y = 0.0
            self._odom_msg.theta = 0.0
            self._odometer_pub.publish(self._odom_msg)

    def _on_loco_rpc(self, msg: RpcReqMsg) -> None:
        result = self._core.handle_rpc(msg.header, msg.body)
        self._emit_actions(result)
        resp = RpcRespMsg()
        resp.uuid = msg.uuid
        resp.header = result.response.header
        resp.body = result.response.body
        self._loco_resp_pub.publish(resp)

    def _on_api_rpc(self, msg: BoosterApiReqMsg) -> None:
        result = self._core.handle_api_call(int(msg.api_id), msg.body)
        self._emit_actions(result)
        resp = BoosterApiRespMsg()
        resp.status = int(result.response.status)
        resp.body = result.response.body
        self._api_resp_pub.publish(resp)

    def _on_kick_ball(self, msg: Any) -> None:
        try:
            self._core.update_kick_reference(
                ball_x=msg.x,
                ball_y=msg.y,
                direction=msg.dir,
                goal_x=msg.goal_x,
                goal_y=msg.goal_y,
                robot_theta_to_field=msg.robot_theta_to_field,
                power=msg.power,
            )
        except Exception as exc:  # noqa: BLE001
            self._node.get_logger().warning(
                f"Rejected /kick_ball telemetry for {self._robot_name}: {exc}"
            )


class BoosterBridgeNode(Node):
    """Single- or multi-robot Booster protocol bridge avoiding 500 Hz /clock overhead."""

    def __init__(self, *, node_name: str = "pumasim_booster_bridge", **kwargs: Any) -> None:
        super().__init__(node_name, **kwargs)
        self.declare_parameter("robot_name", "blue_1")
        self.declare_parameter("robot_names_csv", "")
        self.declare_parameter("robot_type", "auto")
        self.declare_parameter("booster_namespace", "")
        self.declare_parameter("localization_topic", "")
        self.declare_parameter("layout", "auto")
        self.declare_parameter("relay_rgb", True)
        self.declare_parameter("relay_depth", False)
        self.declare_parameter("relay_camera_info", False)
        self.declare_parameter("enable_kick_reference", True)
        self.declare_parameter("vx_min", -0.5)
        self.declare_parameter("vx_max", 1.0)
        self.declare_parameter("vy_min", -0.5)
        self.declare_parameter("vy_max", 0.5)
        self.declare_parameter("wz_min", -1.0)
        self.declare_parameter("wz_max", 1.0)

        csv_robots = [
            r.strip().strip("/")
            for r in str(self.get_parameter("robot_names_csv").value).split(",")
            if r.strip()
        ]
        single_robot = str(self.get_parameter("robot_name").value).strip("/")
        robots = csv_robots or [single_robot]

        robot_type = str(self.get_parameter("robot_type").value)
        booster_ns = str(self.get_parameter("booster_namespace").value)
        loc_topic = str(self.get_parameter("localization_topic").value).strip()
        layout = str(self.get_parameter("layout").value).strip().lower()
        relay_rgb = bool(self.get_parameter("relay_rgb").value)
        relay_depth = bool(self.get_parameter("relay_depth").value)
        relay_camera_info = bool(self.get_parameter("relay_camera_info").value)
        enable_kick_ref = bool(self.get_parameter("enable_kick_reference").value)

        limits = {
            "vx": (
                float(self.get_parameter("vx_min").value),
                float(self.get_parameter("vx_max").value),
            ),
            "vy": (
                float(self.get_parameter("vy_min").value),
                float(self.get_parameter("vy_max").value),
            ),
            "wz": (
                float(self.get_parameter("wz_min").value),
                float(self.get_parameter("wz_max").value),
            ),
        }

        kick_cls = _try_import_kick_msg() if enable_kick_ref else None
        if enable_kick_ref and kick_cls is None:
            self.get_logger().warning(
                "brain.msg.Kick unavailable; /kick_ball telemetry listener skipped"
            )

        self._endpoints: list[_RobotBridgeEndpoint] = []
        for rname in robots:
            if len(robots) > 1:
                ns = f"/{rname}/booster"
                ltopic = f"/{rname}/ground_truth/localization"
            else:
                if booster_ns:
                    ns = booster_ns
                elif layout == "namespaced":
                    ns = f"/{rname}/booster"
                else:
                    ns = ""
                ltopic = loc_topic or f"/{rname}/ground_truth/localization"
            self._endpoints.append(
                _RobotBridgeEndpoint(
                    self,
                    robot_name=rname,
                    robot_type=robot_type,
                    booster_namespace=ns,
                    localization_topic=ltopic,
                    relay_rgb=relay_rgb,
                    relay_depth=relay_depth,
                    relay_camera_info=relay_camera_info,
                    enable_kick_reference=enable_kick_ref,
                    limits=limits,
                    kick_msg_cls=kick_cls,
                )
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = BoosterBridgeNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
