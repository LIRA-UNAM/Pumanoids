"""Separate-process ROS 2 integration tests: PumaSim -> pumasim_booster_bridge -> Booster consumer."""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest

_RUN_E2E = os.environ.get("PUMASIM_TEST_ROS2") == "1"
_PUMASIM_ROOT = Path(
    os.environ.get("PUMASIM_ROOT", str(Path(__file__).resolve().parents[5] / "PumaSim"))
)


def _poll_until(predicate, *, timeout: float = 15.0, step: float = 0.05) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(step)
    raise AssertionError("timed out waiting for separate-process ROS condition")


def _wrap_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _local_ros_env(monkeypatch, domain_id: str) -> dict[str, str]:
    sys.path.insert(0, str(_PUMASIM_ROOT / "tools"))
    from ros2_local import local_environment

    env = local_environment(dict(os.environ))
    env["ROS_DOMAIN_ID"] = domain_id
    for key in set(os.environ) - set(env):
        monkeypatch.delenv(key, raising=False)
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    return env


@pytest.mark.skipif(not _RUN_E2E, reason="opt-in real separate-process ROS 2 test")
@pytest.mark.parametrize(
    ("scenario", "expected_model", "expected_parallel", "expected_serial"),
    [
        ("single_t1", "T1", 0, 23),
        ("single_k1", "K1", 22, 22),
    ],
)
def test_single_robot_full_pipeline_via_external_bridge(
    monkeypatch,
    tmp_path: Path,
    scenario: str,
    expected_model: str,
    expected_parallel: int,
    expected_serial: int,
) -> None:
    import rclpy
    from booster_interface.msg import (
        BoosterApiReqMsg,
        BoosterApiRespMsg,
        LowState,
        Odometer,
        RawBytesMsg,
    )
    from booster_msgs.msg import RpcReqMsg, RpcRespMsg
    from geometry_msgs.msg import Pose
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image

    domain_id = str(150 + (os.getpid() + (0 if scenario == "single_t1" else 1)) % 20)
    env = _local_ros_env(monkeypatch, domain_id)
    env["MUJOCO_GL"] = "egl"

    sim_cmd = [
        "uv",
        "run",
        "--project",
        str(_PUMASIM_ROOT),
        "pumanoids-sim",
        "run",
        "--config",
        scenario,
        "--headless",
        "--ros2",
        "--duration",
        "40",
    ]
    bridge_cmd = [
        "ros2",
        "run",
        "pumasim_booster_bridge",
        "booster_bridge",
        "--ros-args",
        "-p",
        "robot_name:=blue_1",
        "-p",
        "relay_rgb:=true",
        "-p",
        "relay_camera_info:=false",
    ]

    sim_proc = subprocess.Popen(
        sim_cmd,
        cwd=str(_PUMASIM_ROOT),
        env=env,
    )
    bridge_proc = subprocess.Popen(
        bridge_cmd,
        env=env,
    )

    rclpy.init()
    node = rclpy.create_node("booster_e2e_observer")
    received = {
        "clock": 0,
        "low_state": None,
        "odometer_state": None,
        "gt_loc": None,
        "head_pose": None,
        "recovery": None,
        "rgb": None,
        "camera_info": None,
        "loco_resps": {},
        "api_resps": [],
    }
    head_orientations: list[float] = []
    odom_samples: list[tuple[float, float]] = []

    best_effort_1 = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )
    reliable_10 = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    try:
        node.create_subscription(Clock, "/clock", lambda m: received.__setitem__("clock", received["clock"] + 1), best_effort_1)
        node.create_subscription(LowState, "/low_state", lambda m: received.__setitem__("low_state", m), best_effort_1)
        def _on_odom(m: Odometer) -> None:
            received["odometer_state"] = m
            odom_samples.append((m.x, m.y))
        node.create_subscription(Odometer, "/odometer_state", _on_odom, best_effort_1)
        node.create_subscription(Odometer, "/blue_1/ground_truth/localization", lambda m: received.__setitem__("gt_loc", m), reliable_10)
        def _on_head(m: Pose) -> None:
            received["head_pose"] = m
            q = m.orientation
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            head_orientations.append(pitch)
        node.create_subscription(Pose, "/head_pose", _on_head, best_effort_1)
        node.create_subscription(RawBytesMsg, "/fall_down_recovery_state", lambda m: received.__setitem__("recovery", m), reliable_10)
        node.create_subscription(Image, "/boostercamera/head/rgb", lambda m: received.__setitem__("rgb", m), reliable_10)
        # CameraInfo comes directly from PumaSim generic topic via remap (BEST_EFFORT)
        node.create_subscription(CameraInfo, "/blue_1/camera/camera_info", lambda m: received.__setitem__("camera_info", m), best_effort_1)
        node.create_subscription(
            RpcRespMsg,
            "/LocoApiTopicResp",
            lambda m: received["loco_resps"].__setitem__(m.uuid, (json.loads(m.header)["status"], m.body)),
            reliable_10,
        )
        node.create_subscription(BoosterApiRespMsg, "/booster_rpc_resp", lambda m: received["api_resps"].append((m.status, m.body)), reliable_10)

        loco_pub = node.create_publisher(RpcReqMsg, "/LocoApiTopicReq", reliable_10)
        api_pub = node.create_publisher(BoosterApiReqMsg, "/booster_rpc_req", reliable_10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin_until(cond, label="condition", timeout=25.0, detail=None):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if cond():
                    return
            missing = [
                k
                for k in (
                    "low_state",
                    "odometer_state",
                    "gt_loc",
                    "head_pose",
                    "recovery",
                    "rgb",
                    "camera_info",
                )
                if received[k] is None
            ]
            extra = f" detail={detail()}" if detail is not None else ""
            raise AssertionError(
                f"timed out waiting for {label}; clock={received['clock']} missing={missing}{extra}"
            )

        spin_until(
            lambda: all(
                received[k] is not None
                for k in ("low_state", "odometer_state", "gt_loc", "head_pose", "recovery", "rgb", "camera_info")
            ),
            label="initial Booster state & camera streams",
        )

        low: LowState = received["low_state"]
        assert len(low.motor_state_parallel) == expected_parallel
        assert len(low.motor_state_serial) == expected_serial
        assert received["rgb"].encoding == "rgb8"
        assert received["camera_info"].width == 640
        assert bytes(received["recovery"].msg)[1] == 0  # is_recovery_available == 0

        def call_loco(api_id: int, body: dict | str) -> tuple[int, str]:
            uid = str(uuid.uuid4())
            req = RpcReqMsg()
            req.uuid = uid
            req.header = json.dumps({"api_id": api_id})
            req.body = body if isinstance(body, str) else json.dumps(body)
            loco_pub.publish(req)
            spin_until(lambda: uid in received["loco_resps"], timeout=5.0)
            return received["loco_resps"][uid]

        # Query robot info + mode
        st, body = call_loco(2022, {})
        assert st == 0 and json.loads(body)["model"] == expected_model
        st, body = call_loco(2017, {})
        assert st == 0 and json.loads(body)["mode"] == 2

        # Honest 501 refusal for GetUp (2008) and VisualKick (2038)
        st, _ = call_loco(2008, {})
        assert st == 501
        st, _ = call_loco(2038, {"start": True})
        assert st == 501

        # Topic service equivalent on /booster_rpc_req
        api_req = BoosterApiReqMsg()
        api_req.api_id = 2018
        api_req.body = "{}"
        api_pub.publish(api_req)
        spin_until(lambda: len(received["api_resps"]) >= 1, timeout=5.0)
        assert received["api_resps"][-1][0] == 0

        # Command walking + head pitch during walk (T1 policy deadzone sits near 0.4 m/s; 0.6 m/s produces clear walk)
        start_x, start_y = received["gt_loc"].x, received["gt_loc"].y
        st, _ = call_loco(2001, {"vx": 0.6, "vy": 0.0, "vyaw": 0.0})
        assert st == 0
        st, _ = call_loco(2004, {"pitch": 0.45, "yaw": 0.2})
        assert st == 0

        head_before = head_orientations[-1]
        spin_until(
            lambda: (
                math.hypot(received["gt_loc"].x - start_x, received["gt_loc"].y - start_y) > 0.08
                and received["odometer_state"].x > 0.08
                and (
                    abs(received["head_pose"].orientation.y) > 0.05
                    or abs(head_orientations[-1] - head_before) > 0.05
                )
            ),
            label="walking displacement and head actuation",
            timeout=15.0,
            detail=lambda: (
                round(math.hypot(received["gt_loc"].x - start_x, received["gt_loc"].y - start_y), 4),
                round(received["odometer_state"].x, 4),
                round(received["odometer_state"].y, 4),
                round(head_before, 4),
                round(head_orientations[-1], 4),
            ),
        )

        # ResetOdometry resets /odometer_state without altering /blue_1/ground_truth/localization
        gt_before_reset = (received["gt_loc"].x, received["gt_loc"].y)
        st, _ = call_loco(2031, {})
        assert st == 0
        spin_until(
            lambda: math.hypot(received["odometer_state"].x, received["odometer_state"].y) < 0.08,
            label="odometry reset to near-zero",
            timeout=3.0,
        )
        assert math.hypot(received["gt_loc"].x - gt_before_reset[0], received["gt_loc"].y - gt_before_reset[1]) < 0.5
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for proc in (bridge_proc, sim_proc):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()


@pytest.mark.skipif(not _RUN_E2E, reason="opt-in real separate-process ROS 2 test")
def test_multi_robot_t1_k1_isolation_via_external_bridge(monkeypatch) -> None:
    import rclpy
    from booster_interface.msg import LowState, Odometer
    from booster_msgs.msg import RpcReqMsg, RpcRespMsg
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    domain_id = str(175 + os.getpid() % 15)
    env = _local_ros_env(monkeypatch, domain_id)

    sim_cmd = [
        "uv",
        "run",
        "--project",
        str(_PUMASIM_ROOT),
        "pumanoids-sim",
        "run",
        "--config",
        str(_PUMASIM_ROOT / "tests" / "fixtures" / "mixed_t1_k1.yaml"),
        "--headless",
        "--ros2",
        "--duration",
        "20",
    ]
    bridge_blue = [
        "ros2",
        "run",
        "pumasim_booster_bridge",
        "booster_bridge",
        "--ros-args",
        "-r",
        "__node:=booster_bridge_blue_1",
        "-p",
        "robot_name:=blue_1",
        "-p",
        "booster_namespace:=/blue_1/booster",
        "-p",
        "relay_rgb:=false",
    ]
    bridge_red = [
        "ros2",
        "run",
        "pumasim_booster_bridge",
        "booster_bridge",
        "--ros-args",
        "-r",
        "__node:=booster_bridge_red_1",
        "-p",
        "robot_name:=red_1",
        "-p",
        "booster_namespace:=/red_1/booster",
        "-p",
        "relay_rgb:=false",
    ]

    sim_proc = subprocess.Popen(sim_cmd, cwd=str(_PUMASIM_ROOT), env=env)
    b1_proc = subprocess.Popen(bridge_blue, env=env)
    r1_proc = subprocess.Popen(bridge_red, env=env)

    rclpy.init()
    node = rclpy.create_node("multi_booster_observer")
    locs: dict[str, Odometer] = {}
    low_states: dict[str, LowState] = {}
    resps: dict[str, dict[str, tuple[int, str]]] = {"blue_1": {}, "red_1": {}}
    reliable_10 = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

    try:
        node.create_subscription(Odometer, "/blue_1/ground_truth/localization", lambda m: locs.__setitem__("blue_1", m), reliable_10)
        node.create_subscription(Odometer, "/red_1/ground_truth/localization", lambda m: locs.__setitem__("red_1", m), reliable_10)
        node.create_subscription(LowState, "/blue_1/booster/low_state", lambda m: low_states.__setitem__("blue_1", m), reliable_10)
        node.create_subscription(LowState, "/red_1/booster/low_state", lambda m: low_states.__setitem__("red_1", m), reliable_10)
        node.create_subscription(
            RpcRespMsg,
            "/blue_1/booster/LocoApiTopicResp",
            lambda m: resps["blue_1"].__setitem__(m.uuid, (json.loads(m.header)["status"], m.body)),
            reliable_10,
        )
        node.create_subscription(
            RpcRespMsg,
            "/red_1/booster/LocoApiTopicResp",
            lambda m: resps["red_1"].__setitem__(m.uuid, (json.loads(m.header)["status"], m.body)),
            reliable_10,
        )
        pub_b = node.create_publisher(RpcReqMsg, "/blue_1/booster/LocoApiTopicReq", reliable_10)
        pub_r = node.create_publisher(RpcReqMsg, "/red_1/booster/LocoApiTopicReq", reliable_10)

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        def spin_until(cond, timeout=20.0):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if cond():
                    return
            raise AssertionError("timed out waiting for multi-robot bridge condition")

        spin_until(lambda: "blue_1" in locs and "red_1" in locs and "blue_1" in low_states and "red_1" in low_states)
        # mixed_t1_k1 spawns blue_1 at (-1.0, 0, yaw=0) and red_1 at (+1.0, 0, yaw=pi)
        assert locs["blue_1"].x == pytest.approx(-1.0, abs=0.1)
        assert locs["red_1"].x == pytest.approx(1.0, abs=0.1)
        assert abs(locs["red_1"].theta) == pytest.approx(math.pi, abs=0.15)

        uid_b = str(uuid.uuid4())
        req_b = RpcReqMsg(uuid=uid_b, header='{"api_id":2022}', body="{}")
        uid_r = str(uuid.uuid4())
        req_r = RpcReqMsg(uuid=uid_r, header='{"api_id":2022}', body="{}")
        pub_b.publish(req_b)
        pub_r.publish(req_r)
        spin_until(lambda: uid_b in resps["blue_1"] and uid_r in resps["red_1"], timeout=5.0)
        assert json.loads(resps["blue_1"][uid_b][1])["model"] == "T1"
        assert json.loads(resps["red_1"][uid_r][1])["model"] == "K1"
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for proc in (r1_proc, b1_proc, sim_proc):
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
