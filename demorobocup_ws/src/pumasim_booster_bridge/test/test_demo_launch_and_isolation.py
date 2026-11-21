"""Regression & E2E isolation tests for robocup_demo_robot.launch.py (params, GC, TF, camera)."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml
from pumasim_booster_bridge.launch_utils import (
    flatten_ros_parameters,
    load_node_parameters_from_yaml,
)

_RUN_E2E = os.environ.get("PUMASIM_TEST_ROS2") == "1"
_PKG_ROOT = Path(__file__).resolve().parents[1]
_WS_SRC = _PKG_ROOT.parents[0]
_PUMASIM_ROOT = Path(
    os.environ.get("PUMASIM_ROOT", str(Path(__file__).resolve().parents[5] / "PumaSim"))
)
_upstream_install = os.environ.get("PUMASIM_UPSTREAM_DEMO_INSTALL")
_UPSTREAM_DEMO_INSTALL = Path(_upstream_install) if _upstream_install else None


def _local_ros_env(monkeypatch, domain_id: str) -> dict[str, str]:
    sys.path.insert(0, str(_PUMASIM_ROOT / "tools"))
    from ros2_local import local_environment

    env = local_environment(dict(os.environ))
    env["ROS_DOMAIN_ID"] = domain_id
    env["MUJOCO_GL"] = "egl"
    env["PYTHONUNBUFFERED"] = "1"
    for key in set(os.environ) - set(env):
        monkeypatch.delenv(key, raising=False)
    for key, value in env.items():
        monkeypatch.setenv(key, value)
    return env


def _poll_until(predicate, *, timeout: float = 25.0, step: float = 0.05, detail=None) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(step)
    extra = f" ({detail()})" if detail is not None else ""
    raise AssertionError(f"timed out waiting for separate-process ROS condition{extra}")


def _load_launch_module():
    import importlib.util

    launch_path = _PKG_ROOT / "launch" / "robocup_demo_robot.launch.py"
    spec = importlib.util.spec_from_file_location("robocup_demo_robot_launch", launch_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_dummy_brain_script(path: Path) -> None:
    """Write a standalone Python ROS 2 node mirroring Pumanoids brain.cpp param/sub/TF behavior."""

    path.write_text(
        """
import json
import sys
import rclpy
from game_controller_interface.msg import GameControlData
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class DummyBrainNode(Node):
    def __init__(self) -> None:
        super().__init__("brain_node")
        # Exact defaults matching Pumanoids brain/src/brain.cpp so YAML loads are tested live.
        self.declare_parameter("game.team_id", 0)
        self.declare_parameter("game.player_id", 1)
        self.declare_parameter("game.field_type", "")
        self.declare_parameter("game.player_role", "")
        self.declare_parameter("robot.robot_height", 0.90)
        self.declare_parameter("robot.odom_factor", 1.2)
        self.declare_parameter("robot.vx_factor", 0.5)
        self.declare_parameter("strategy.soft_kickoff", False)
        self.declare_parameter("vision.image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("vision.depth_image_topic", "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("vision.cam_pixel_width", 1280.0)
        self.declare_parameter("vision.cam_pixel_height", 720.0)
        self.declare_parameter("tf_x", 0.0)

        self._team_id = int(self.get_parameter("game.team_id").value)
        self._player_id = int(self.get_parameter("game.player_id").value)
        self._tf_x = float(self.get_parameter("tf_x").value)

        self._gc_pub = self.create_publisher(String, "~/gc_seen", 10)
        self.create_subscription(
            GameControlData, "/robocup/game_controller", self._on_gc, 10
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self._publish_tf)

    def _on_gc(self, msg: GameControlData) -> None:
        payload = {
            "node": self.get_name(),
            "team_id": self._team_id,
            "player_id": self._player_id,
            "state": int(msg.state),
            "kick_off_team": int(msg.kick_off_team),
        }
        self._gc_pub.publish(String(data=json.dumps(payload)))

    def _publish_tf(self) -> None:
        # Mirrors Pumanoids brain.cpp: fixed frame names odom -> base_link!
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self._tf_x
        t.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(t)


def main() -> None:
    rclpy.init(args=sys.argv)
    node = DummyBrainNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
""",
        encoding="utf-8",
    )


def _dump_ros_params_yaml(path: Path, node_name_key: str, params: dict[str, object]) -> None:
    path.write_text(
        yaml.safe_dump({node_name_key: {"ros__parameters": params}}, sort_keys=False),
        encoding="utf-8",
    )


def _ros2_param_get(node_name: str, param_name: str, env: dict[str, str]) -> str:
    for _ in range(3):
        res = subprocess.run(
            [
                "ros2",
                "param",
                "get",
                "--no-daemon",
                "--spin-time",
                "2",
                "--timeout",
                "3",
                node_name,
                param_name,
            ],
            env=env,
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        if res.returncode == 0 and res.stdout.strip():
            return res.stdout.strip()
    return ""


def _terminate_tree(proc: subprocess.Popen) -> None:
    try:
        pgid = os.getpgid(proc.pid)
        if pgid != os.getpgrp():
            os.killpg(pgid, signal.SIGTERM)
        else:
            proc.terminate()
    except OSError:
        proc.terminate()
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            pgid = os.getpgid(proc.pid)
            if pgid != os.getpgrp():
                os.killpg(pgid, signal.SIGKILL)
            else:
                proc.kill()
        except OSError:
            proc.kill()
        proc.wait(timeout=5)


def test_launch_utils_flatten_and_precedence() -> None:
    base = load_node_parameters_from_yaml(_WS_SRC / "brain" / "config" / "config.yaml")
    t1 = load_node_parameters_from_yaml(_PKG_ROOT / "config" / "brain_t1.yaml")
    k1 = load_node_parameters_from_yaml(_PKG_ROOT / "config" / "brain_k1.yaml")

    assert base["game.field_type"] == "robo_league"
    assert base["strategy.soft_kickoff"] is True
    assert base["vision.cam_pixel_width"] == 1280.0

    merged_t1 = {**base, **t1, "game.team_id": 70, "game.player_id": 2}
    merged_k1 = {**base, **k1, "game.team_id": 71, "game.player_id": 3}

    assert merged_t1["game.field_type"] == "adult_size"
    assert merged_t1["game.number_of_players"] == 1
    assert merged_t1["game_control_ip"] == "127.0.0.1"
    assert merged_t1["robot.robot_height"] == pytest.approx(0.665)
    assert merged_k1["robot.robot_height"] == pytest.approx(0.5125)
    assert merged_t1["vision.cam_pixel_width"] == pytest.approx(640.0)
    assert merged_t1["strategy.soft_kickoff"] is True
    assert merged_t1["game.team_id"] == 70
    assert merged_k1["game.team_id"] == 71

    assert flatten_ros_parameters({"a": {"b": 1}}) == {"a.b": 1}


def test_build_demo_remappings_shared_gc_and_tf_isolation() -> None:
    mod = _load_launch_module()

    shared_only = mod.build_demo_remappings(
        robot_name="blue_1",
        booster_namespace="/blue_1/booster",
        rgb_target="/blue_1/booster/boostercamera/head/rgb",
        camera_info_source="/blue_1/camera/camera_info",
        depth_target="/blue_1/camera/depth/image_raw",
        isolate_tf=True,
    )
    remap_dict = dict(shared_only)

    assert "/robocup/game_controller" not in remap_dict
    assert remap_dict["/tf"] == "/blue_1/booster/tf"
    assert remap_dict["tf"] == "/blue_1/booster/tf"
    assert remap_dict["/tf_static"] == "/blue_1/booster/tf_static"
    assert remap_dict["/boostercamera/head/rgb/camera_info"] == "/blue_1/camera/camera_info"
    assert remap_dict["/boostercamera/head/rgb"] == "/blue_1/booster/boostercamera/head/rgb"

    single = dict(
        mod.build_demo_remappings(
            robot_name="blue_1",
            booster_namespace="",
            rgb_target="/boostercamera/head/rgb",
            camera_info_source="/blue_1/camera/camera_info",
            depth_target="/blue_1/camera/depth/image_raw",
            isolate_tf=False,
        )
    )
    assert "/tf" not in single


@pytest.mark.skipif(not _RUN_E2E, reason="opt-in real separate-process ROS 2 test")
def test_brain_parameter_name_mismatch_regression_and_dual_instance_param_get(
    monkeypatch, tmp_path: Path
) -> None:
    env = _local_ros_env(monkeypatch, str(160 + (os.getpid() % 15)))
    script = tmp_path / "dummy_brain.py"
    _write_dummy_brain_script(script)

    base_yaml = _WS_SRC / "brain" / "config" / "config.yaml"
    t1_yaml = _PKG_ROOT / "config" / "brain_t1.yaml"
    k1_yaml = _PKG_ROOT / "config" / "brain_k1.yaml"

    # 1) Regression check: passing raw brain_node-keyed YAMLs under __node:=brain_blue_1
    # must fail to populate game.field_type / robot.odom_factor / vision.cam_pixel_width.
    broken = subprocess.Popen(
        [
            sys.executable,
            str(script),
            "--ros-args",
            "-r",
            "__node:=brain_blue_1",
            "--params-file",
            str(base_yaml),
            "--params-file",
            str(t1_yaml),
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        _poll_until(
            lambda: "String value is:" in _ros2_param_get("/brain_blue_1", "game.field_type", env),
            timeout=15.0,
            step=0.25,
        )
        assert _ros2_param_get("/brain_blue_1", "game.field_type", env) == "String value is:"
        assert "1.2" in _ros2_param_get("/brain_blue_1", "robot.odom_factor", env)
        assert "1280.0" in _ros2_param_get("/brain_blue_1", "vision.cam_pixel_width", env)
    finally:
        _terminate_tree(broken)

    # 2) Fixed path: flattened base -> overlay -> launch args under actual node names
    # Verified with two simultaneous instances (blue_1 T1 vs red_1 K1) using ros2 param get.
    base_params = load_node_parameters_from_yaml(base_yaml)
    t1_params = load_node_parameters_from_yaml(t1_yaml)
    k1_params = load_node_parameters_from_yaml(k1_yaml)

    base_flat = tmp_path / "base_flat.yaml"
    t1_flat = tmp_path / "t1_flat.yaml"
    k1_flat = tmp_path / "k1_flat.yaml"
    blue_inst = tmp_path / "blue_inst.yaml"
    red_inst = tmp_path / "red_inst.yaml"
    _dump_ros_params_yaml(base_flat, "/**", base_params)
    _dump_ros_params_yaml(t1_flat, "/**", t1_params)
    _dump_ros_params_yaml(k1_flat, "/**", k1_params)
    _dump_ros_params_yaml(
        blue_inst,
        "brain_blue_1",
        {
            "game.team_id": 70,
            "game.player_id": 1,
            "game.player_role": "striker",
            "vision.image_topic": "/blue_1/booster/boostercamera/head/rgb",
        },
    )
    _dump_ros_params_yaml(
        red_inst,
        "brain_red_1",
        {
            "game.team_id": 71,
            "game.player_id": 3,
            "game.player_role": "goal_keeper",
            "vision.image_topic": "/red_1/booster/boostercamera/head/rgb",
        },
    )

    blue = subprocess.Popen(
        [
            sys.executable,
            str(script),
            "--ros-args",
            "-r",
            "__node:=brain_blue_1",
            "--params-file",
            str(base_flat),
            "--params-file",
            str(t1_flat),
            "--params-file",
            str(blue_inst),
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    red = subprocess.Popen(
        [
            sys.executable,
            str(script),
            "--ros-args",
            "-r",
            "__node:=brain_red_1",
            "--params-file",
            str(base_flat),
            "--params-file",
            str(k1_flat),
            "--params-file",
            str(red_inst),
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        for node_name, expected in [
            (
                "/brain_blue_1",
                {
                    "game.field_type": "String value is: adult_size",
                    "robot.odom_factor": "Double value is: 1.0",
                    "robot.robot_height": "Double value is: 0.665",
                    "vision.cam_pixel_width": "Double value is: 640.0",
                    "vision.cam_pixel_height": "Double value is: 480.0",
                    "game.team_id": "Integer value is: 70",
                    "game.player_id": "Integer value is: 1",
                    "game.player_role": "String value is: striker",
                    "vision.image_topic": "String value is: /blue_1/booster/boostercamera/head/rgb",
                    "strategy.soft_kickoff": "Boolean value is: True",
                },
            ),
            (
                "/brain_red_1",
                {
                    "game.field_type": "String value is: adult_size",
                    "robot.odom_factor": "Double value is: 1.0",
                    "robot.robot_height": "Double value is: 0.5125",
                    "vision.cam_pixel_width": "Double value is: 640.0",
                    "vision.cam_pixel_height": "Double value is: 480.0",
                    "game.team_id": "Integer value is: 71",
                    "game.player_id": "Integer value is: 3",
                    "game.player_role": "String value is: goal_keeper",
                    "vision.image_topic": "String value is: /red_1/booster/boostercamera/head/rgb",
                    "strategy.soft_kickoff": "Boolean value is: True",
                },
            ),
        ]:
            for param, want in expected.items():
                _poll_until(
                    lambda n=node_name, p=param, w=want: _ros2_param_get(n, p, env) == w,
                    timeout=15.0,
                    step=0.25,
                )
    finally:
        blue.terminate()
        red.terminate()
        blue.wait(timeout=8)
        red.wait(timeout=8)


@pytest.mark.skipif(not _RUN_E2E, reason="opt-in real separate-process ROS 2 test")
def test_two_robot_shared_gc_tf_isolation_and_camera_relay(
    monkeypatch, tmp_path: Path
) -> None:
    import json

    import rclpy
    from game_controller_interface.msg import GameControlData
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    env = _local_ros_env(monkeypatch, str(161 + (os.getpid() % 15)))
    mod = _load_launch_module()
    script = tmp_path / "dummy_brain.py"
    _write_dummy_brain_script(script)

    def _remap_cli(pairs: list[tuple[str, str]]) -> list[str]:
        out: list[str] = []
        for src, dst in pairs:
            out.extend(["-r", f"{src}:={dst}"])
        return out

    sim_cfg = tmp_path / "two_robots_cam.yaml"
    sim_cfg.write_text(
        yaml.safe_dump(
            {
                "world": "robocup_msl_2026",
                "simulation": {"headless": True, "realtime": False},
                "robots": [
                    {
                        "name": "blue_1",
                        "type": "t1",
                        "team": "blue",
                        "position": [-1.0, 0.0, 0.0],
                        "yaw": 0.0,
                        "camera": {
                            "enabled": True,
                            "width": 640,
                            "height": 480,
                            "fps": 30,
                            "depth": False,
                        },
                    },
                    {
                        "name": "red_1",
                        "type": "k1",
                        "team": "red",
                        "position": [1.0, 0.0, 0.0],
                        "yaw": 3.141592653589793,
                        "camera": {
                            "enabled": True,
                            "width": 640,
                            "height": 480,
                            "fps": 30,
                            "depth": False,
                        },
                    },
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    sim = subprocess.Popen(
        [
            "uv",
            "run",
            "--locked",
            "--no-sync",
            "--project",
            str(_PUMASIM_ROOT),
            "pumanoids-sim",
            "run",
            "--config",
            str(sim_cfg),
            "--headless",
            "--ros2",
            "--duration",
            "45",
        ],
        cwd=str(_PUMASIM_ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    blue_launch = subprocess.Popen(
        [
            "ros2",
            "launch",
            "pumasim_booster_bridge",
            "robocup_demo_robot.launch.py",
            "robot_name:=blue_1",
            "robot_type:=t1",
            "namespaced:=true",
            "relay_rgb:=true",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    red_launch = subprocess.Popen(
        [
            "ros2",
            "launch",
            "pumasim_booster_bridge",
            "robocup_demo_robot.launch.py",
            "robot_name:=red_1",
            "robot_type:=k1",
            "namespaced:=true",
            "relay_rgb:=true",
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    blue_remaps = mod.build_demo_remappings(
        robot_name="blue_1",
        booster_namespace="/blue_1/booster",
        rgb_target="/blue_1/booster/boostercamera/head/rgb",
        camera_info_source="/blue_1/camera/camera_info",
        depth_target="/blue_1/camera/depth/image_raw",
        isolate_tf=True,
    )
    red_remaps = mod.build_demo_remappings(
        robot_name="red_1",
        booster_namespace="/red_1/booster",
        rgb_target="/red_1/booster/boostercamera/head/rgb",
        camera_info_source="/red_1/camera/camera_info",
        depth_target="/red_1/camera/depth/image_raw",
        isolate_tf=True,
    )

    blue_brain = subprocess.Popen(
        [
            sys.executable,
            str(script),
            "--ros-args",
            "-r",
            "__node:=brain_blue_1",
            "-p",
            "game.team_id:=70",
            "-p",
            "game.player_id:=1",
            "-p",
            "tf_x:=1.25",
            *_remap_cli(blue_remaps),
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    red_brain = subprocess.Popen(
        [
            sys.executable,
            str(script),
            "--ros-args",
            "-r",
            "__node:=brain_red_1",
            "-p",
            "game.team_id:=71",
            "-p",
            "game.player_id:=3",
            "-p",
            "tf_x:=-2.75",
            *_remap_cli(red_remaps),
        ],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    obs = rclpy.create_node("isolation_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(obs)

    best_effort = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=5,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )
    reliable = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )

    gc_pub = obs.create_publisher(GameControlData, "/robocup/game_controller", 10)
    gc_seen: dict[str, dict] = {}
    tf_global: list[tuple[str, str, float]] = []
    tf_blue: list[tuple[str, str, float]] = []
    tf_red: list[tuple[str, str, float]] = []
    raw_stamps: dict[str, set[tuple[int, int]]] = {"blue_1": set(), "red_1": set()}
    relay_msgs: dict[str, list[tuple[str, int, int]]] = {"blue_1": [], "red_1": []}
    cam_info_frames: dict[str, list[str]] = {"blue_1": [], "red_1": []}

    obs.create_subscription(
        String,
        "/brain_blue_1/gc_seen",
        lambda m: gc_seen.__setitem__("blue_1", json.loads(m.data)),
        10,
    )
    obs.create_subscription(
        String,
        "/brain_red_1/gc_seen",
        lambda m: gc_seen.__setitem__("red_1", json.loads(m.data)),
        10,
    )
    obs.create_subscription(
        TFMessage,
        "/tf",
        lambda m: tf_global.extend(
            (t.header.frame_id, t.child_frame_id, t.transform.translation.x)
            for t in m.transforms
        ),
        50,
    )
    obs.create_subscription(
        TFMessage,
        "/blue_1/booster/tf",
        lambda m: tf_blue.extend(
            (t.header.frame_id, t.child_frame_id, t.transform.translation.x)
            for t in m.transforms
        ),
        50,
    )
    obs.create_subscription(
        TFMessage,
        "/red_1/booster/tf",
        lambda m: tf_red.extend(
            (t.header.frame_id, t.child_frame_id, t.transform.translation.x)
            for t in m.transforms
        ),
        50,
    )
    for robot in ("blue_1", "red_1"):
        obs.create_subscription(
            Image,
            f"/{robot}/camera/image_raw",
            lambda m, r=robot: raw_stamps[r].add(
                (m.header.stamp.sec, m.header.stamp.nanosec)
            ),
            best_effort,
        )
        obs.create_subscription(
            Image,
            f"/{robot}/booster/boostercamera/head/rgb",
            lambda m, r=robot: relay_msgs[r].append(
                (m.header.frame_id, m.header.stamp.sec, m.header.stamp.nanosec)
            ),
            reliable,
        )
        obs.create_subscription(
            CameraInfo,
            f"/{robot}/camera/camera_info",
            lambda m, r=robot: cam_info_frames[r].append(m.header.frame_id),
            best_effort,
        )

    def _step() -> bool:
        msg = GameControlData()
        msg.state = 3
        msg.kick_off_team = 70
        gc_pub.publish(msg)
        executor.spin_once(timeout_sec=0.05)
        return (
            "blue_1" in gc_seen
            and "red_1" in gc_seen
            and len(tf_global) >= 2
            and len(tf_blue) >= 2
            and len(tf_red) >= 2
            and len(relay_msgs["blue_1"]) >= 2
            and len(relay_msgs["red_1"]) >= 2
            and len(cam_info_frames["blue_1"]) >= 1
            and len(cam_info_frames["red_1"]) >= 1
        )

    try:
        _poll_until(
            _step,
            timeout=30.0,
            step=0.02,
            detail=lambda: (
                f"nodes={obs.get_node_names()} gc={list(gc_seen)} "
                f"tf_global={len(tf_global)} tf_blue={len(tf_blue)} tf_red={len(tf_red)} "
                f"relay={{'b': {len(relay_msgs['blue_1'])}, 'r': {len(relay_msgs['red_1'])}}} "
                f"cinfo={{'b': {len(cam_info_frames['blue_1'])}, 'r': {len(cam_info_frames['red_1'])}}}"
            ),
        )
    except Exception:
        for label, proc in (("sim", sim), ("blue_launch", blue_launch), ("red_launch", red_launch)):
            _terminate_tree(proc)
            out = proc.stdout.read() if proc.stdout else ""
            print(f"\n=== {label} ===\n{out[-1200:]}")
        raise

        assert gc_seen["blue_1"]["team_id"] == 70
        assert gc_seen["blue_1"]["player_id"] == 1
        assert gc_seen["blue_1"]["state"] == 3
        assert gc_seen["red_1"]["team_id"] == 71
        assert gc_seen["red_1"]["player_id"] == 3
        assert gc_seen["red_1"]["state"] == 3

        global_pairs = {(p, c) for p, c, _ in tf_global}
        assert ("odom", "base_link") not in global_pairs
        assert ("blue_1/base_link", "blue_1/camera_optical_frame") in global_pairs
        assert ("red_1/base_link", "red_1/camera_optical_frame") in global_pairs

        assert { (p, c) for p, c, _ in tf_blue } == {("odom", "base_link")}
        assert all(x == pytest.approx(1.25) for _, _, x in tf_blue)
        assert { (p, c) for p, c, _ in tf_red } == {("odom", "base_link")}
        assert all(x == pytest.approx(-2.75) for _, _, x in tf_red)

        assert all(fid == "blue_1/camera_optical_frame" for fid in cam_info_frames["blue_1"])
        assert all(fid == "red_1/camera_optical_frame" for fid in cam_info_frames["red_1"])
        for robot in ("blue_1", "red_1"):
            assert all(fid == f"{robot}/camera_optical_frame" for fid, _, _ in relay_msgs[robot])
            assert any((s, ns) in raw_stamps[robot] for _, s, ns in relay_msgs[robot])
    finally:
        executor.shutdown()
        obs.destroy_node()
        rclpy.shutdown()
        for proc in (blue_brain, red_brain, blue_launch, red_launch, sim):
            _terminate_tree(proc)


@pytest.mark.skipif(
    not (
        _RUN_E2E
        and _UPSTREAM_DEMO_INSTALL is not None
        and (_UPSTREAM_DEMO_INSTALL / "brain" / "lib" / "brain" / "brain_node").is_file()
    ),
    reason="set PUMASIM_UPSTREAM_DEMO_INSTALL to the prebuilt official demo overlay",
)
def test_real_robocup_demo_brain_node_launch_and_command(monkeypatch) -> None:
    import rclpy
    from booster_interface.msg import LowState, Odometer
    from booster_msgs.msg import RpcReqMsg
    from game_controller_interface.msg import GameControlData, TeamInfo
    from geometry_msgs.msg import Pose
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState

    # Extend env with upstream robocup_demo Jazzy packages (brain/vision) without bash sourcing overhead.
    env = _local_ros_env(monkeypatch, str(162 + (os.getpid() % 15)))
    demo_env = dict(env)
    upstream_prefixes = [
        str(_UPSTREAM_DEMO_INSTALL / pkg)
        for pkg in ("brain", "vision", "behaviortree_cpp", "backward_ros")
    ]
    demo_env["AMENT_PREFIX_PATH"] = ":".join(
        [demo_env.get("AMENT_PREFIX_PATH", ""), *upstream_prefixes]
    ).strip(":")
    demo_env["LD_LIBRARY_PATH"] = ":".join(
        [
            demo_env.get("LD_LIBRARY_PATH", ""),
            *(f"{p}/lib" for p in upstream_prefixes),
        ]
    ).strip(":")

    sim = subprocess.Popen(
        [
            "uv",
            "run",
            "--locked",
            "--no-sync",
            "--project",
            str(_PUMASIM_ROOT),
            "pumanoids-sim",
            "run",
            "--config",
            "single_t1",
            "--headless",
            "--ros2",
            "--duration",
            "50",
        ],
        cwd=str(_PUMASIM_ROOT),
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    demo = subprocess.Popen(
        [
            "ros2",
            "launch",
            "pumasim_booster_bridge",
            "robocup_demo_robot.launch.py",
            "robot_name:=blue_1",
            "robot_type:=t1",
            "enable_vision:=false",
            "enable_brain:=true",
            "team_id:=70",
            "player_id:=1",
            "player_role:=striker",
        ],
        env=demo_env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    rclpy.init()
    obs = rclpy.create_node("real_brain_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(obs)
    gc_pub = obs.create_publisher(GameControlData, "/robocup/game_controller", 10)
    seen = {"low": 0, "odom": 0, "head": 0, "rpc": 0, "jt": 0}
    obs.create_subscription(LowState, "/low_state", lambda _: seen.__setitem__("low", seen["low"] + 1), 10)
    obs.create_subscription(Odometer, "/odometer_state", lambda _: seen.__setitem__("odom", seen["odom"] + 1), 10)
    obs.create_subscription(Pose, "/head_pose", lambda _: seen.__setitem__("head", seen["head"] + 1), 10)
    obs.create_subscription(RpcReqMsg, "/LocoApiTopicReq", lambda _: seen.__setitem__("rpc", seen["rpc"] + 1), 10)
    obs.create_subscription(JointState, "/blue_1/joint_targets", lambda _: seen.__setitem__("jt", seen["jt"] + 1), 10)

    gc_msg = GameControlData()
    gc_msg.state = 3
    gc_msg.first_half = 1
    gc_msg.kick_off_team = 70
    gc_msg.teams = [TeamInfo(team_number=70), TeamInfo(team_number=71)]

    def _poll_step() -> bool:
        gc_pub.publish(gc_msg)
        executor.spin_once(timeout_sec=0.05)
        return (
            seen["low"] >= 1
            and seen["odom"] >= 1
            and seen["head"] >= 1
            and seen["rpc"] >= 1
            and seen["jt"] >= 1
        )

    try:
        _poll_until(
            _poll_step,
            timeout=30.0,
            step=0.02,
            detail=lambda: f"nodes={obs.get_node_names()} seen={seen}",
        )
        assert _ros2_param_get("/brain_blue_1", "game.field_type", env) == "String value is: adult_size"
        assert _ros2_param_get("/brain_blue_1", "game.player_role", env) == "String value is: striker"
    except Exception:
        for label, proc in (("demo", demo), ("sim", sim)):
            _terminate_tree(proc)
            out = proc.stdout.read() if proc.stdout else ""
            print(f"\n=== {label} ===\n{out[-1200:]}")
        raise
    finally:
        executor.shutdown()
        obs.destroy_node()
        rclpy.shutdown()
        _terminate_tree(demo)
        _terminate_tree(sim)


def _has_pumanoids_sim_demo_binaries() -> bool:
    try:
        from ament_index_python.packages import PackageNotFoundError, get_package_prefix
    except ImportError:
        return False
    for pkg, exe in (("vision", "vision_node"), ("brain", "brain_node")):
        try:
            prefix = Path(get_package_prefix(pkg))
        except PackageNotFoundError:
            return False
        if not (prefix / "lib" / pkg / exe).is_file():
            return False
    return True


@pytest.mark.skipif(
    not (_RUN_E2E and _has_pumanoids_sim_demo_binaries()),
    reason="opt-in ROS 2 E2E requiring built Pumanoids vision_node and brain_node",
)
def test_pumanoids_single_t1_onnx_vision_and_brain_closed_loop(
    monkeypatch, tmp_path: Path
) -> None:
    import json
    import math

    import rclpy
    from booster_msgs.msg import RpcReqMsg, RpcRespMsg
    from game_controller_interface.msg import GameControlData, RobotInfo, TeamInfo
    from geometry_msgs.msg import Pose, Twist
    from nav_msgs.msg import Odometry
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from vision_interface.msg import Detections, LineSegments

    env = _local_ros_env(monkeypatch, str(164 + (os.getpid() % 15)))
    sim_log = tmp_path / "sim.log"
    demo_log = tmp_path / "demo.log"
    sim_fh = open(sim_log, "w", encoding="utf-8")
    demo_fh = open(demo_log, "w", encoding="utf-8")

    sim = subprocess.Popen(
        [
            "uv",
            "run",
            "--locked",
            "--no-sync",
            "--project",
            str(_PUMASIM_ROOT),
            "pumanoids-sim",
            "run",
            "--config",
            "single_t1",
            "--headless",
            "--ros2",
            "--duration",
            "45",
        ],
        cwd=str(_PUMASIM_ROOT),
        env=env,
        stdout=sim_fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    demo = subprocess.Popen(
        [
            "ros2",
            "launch",
            "pumasim_booster_bridge",
            "robocup_demo_robot.launch.py",
            "robot_name:=blue_1",
            "robot_type:=t1",
            "namespaced:=false",
            "enable_vision:=true",
            "enable_brain:=true",
            "team_id:=70",
            "player_id:=1",
            "player_role:=striker",
        ],
        env=env,
        stdout=demo_fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )

    rclpy.init()
    obs = rclpy.create_node("pumanoids_closed_loop_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(obs)
    gc_pub = obs.create_publisher(GameControlData, "/robocup/game_controller", 10)

    det_counts: list[int] = []
    line_counts: list[int] = []
    req_apis: list[int] = []
    resp_statuses: list[int] = []
    cmd_vels: list[tuple[float, float, float]] = []
    jt_count = [0]
    gt_xy: list[tuple[float, float]] = []
    head_quats: list[tuple[float, float, float, float]] = []

    obs.create_subscription(
        Detections,
        "/booster_vision/detection",
        lambda m: det_counts.append(len(m.detected_objects)),
        10,
    )
    obs.create_subscription(
        LineSegments,
        "/booster_vision/line_segments",
        lambda m: line_counts.append(len(m.coordinates) // 4),
        10,
    )
    obs.create_subscription(
        RpcReqMsg,
        "/LocoApiTopicReq",
        lambda m: req_apis.append(int(json.loads(m.header)["api_id"])),
        10,
    )
    obs.create_subscription(
        RpcRespMsg,
        "/LocoApiTopicResp",
        lambda m: resp_statuses.append(int(json.loads(m.header)["status"])),
        10,
    )
    obs.create_subscription(
        Twist,
        "/blue_1/cmd_vel",
        lambda m: cmd_vels.append((m.linear.x, m.linear.y, m.angular.z)),
        10,
    )
    obs.create_subscription(
        JointState,
        "/blue_1/joint_targets",
        lambda _: jt_count.__setitem__(0, jt_count[0] + 1),
        10,
    )
    obs.create_subscription(
        Odometry,
        "/blue_1/ground_truth/odom",
        lambda m: gt_xy.append((m.pose.pose.position.x, m.pose.pose.position.y)),
        10,
    )
    obs.create_subscription(
        Pose,
        "/head_pose",
        lambda m: head_quats.append(
            (m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w)
        ),
        10,
    )

    gc_msg = GameControlData()
    gc_msg.version = 20
    gc_msg.players_per_team = 11
    gc_msg.state = 3
    gc_msg.first_half = 1
    gc_msg.kick_off_team = 70
    gc_msg.teams = [
        TeamInfo(
            team_number=70, goalkeeper=1, players=[RobotInfo() for _ in range(11)]
        ),
        TeamInfo(
            team_number=71, goalkeeper=1, players=[RobotInfo() for _ in range(11)]
        ),
    ]

    def _poll_step() -> bool:
        gc_pub.publish(gc_msg)
        executor.spin_once(timeout_sec=0.05)
        xy_disp = (
            math.hypot(gt_xy[-1][0] - gt_xy[0][0], gt_xy[-1][1] - gt_xy[0][1])
            if len(gt_xy) >= 2
            else 0.0
        )
        head_delta = (
            math.sqrt(sum((a - b) ** 2 for a, b in zip(head_quats[-1], head_quats[0])))
            if len(head_quats) >= 2
            else 0.0
        )
        return (
            len(det_counts) >= 2
            and max(det_counts, default=0) >= 1
            and len(line_counts) >= 2
            and max(line_counts, default=0) >= 1
            and any(api in (2001, 2004) for api in req_apis)
            and any(st == 0 for st in resp_statuses)
            and (len(cmd_vels) >= 1 or jt_count[0] >= 1)
            and (xy_disp > 0.05 or head_delta > 0.05)
        )

    try:
        _poll_until(
            _poll_step,
            timeout=35.0,
            step=0.02,
            detail=lambda: (
                f"dets={len(det_counts)} lines={len(line_counts)} "
                f"apis={sorted(set(req_apis))} cmd_vels={len(cmd_vels)} jts={jt_count[0]}"
            ),
        )
        assert _ros2_param_get("/brain_blue_1", "game.field_type", env) == "String value is: adult_size"
        assert _ros2_param_get("/brain_blue_1", "game.player_role", env) == "String value is: striker"
        assert _ros2_param_get("/brain_blue_1", "game.number_of_players", env) == "Integer value is: 1"
    except Exception:
        for proc in (demo, sim):
            _terminate_tree(proc)
        for label, p in (("demo", demo_log), ("sim", sim_log)):
            txt = p.read_text(encoding="utf-8", errors="replace") if p.is_file() else ""
            print(f"\n=== {label} ===\n{txt[-1500:]}")
        raise
    finally:
        executor.shutdown()
        obs.destroy_node()
        rclpy.shutdown()
        _terminate_tree(demo)
        _terminate_tree(sim)
        sim_fh.close()
        demo_fh.close()


def _eval_launch_node(launch_path: Path, args: dict[str, str]):
    import importlib.util
    import yaml

    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    from launch_ros.actions import Node

    spec = importlib.util.spec_from_file_location(launch_path.stem, launch_path)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    nodes = []
    for entity in ld.entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.visit(ctx)
        elif isinstance(entity, OpaqueFunction):
            ctx.launch_configurations.update(args)
            res = entity.visit(ctx)
            if res:
                for n in res:
                    if isinstance(n, Node):
                        n._perform_substitutions(ctx)
                        param_files: list[str] = []
                        param_dicts: list[dict] = []
                        for p_path, _ in getattr(n, "_Node__expanded_parameter_arguments", []) or []:
                            p_str = str(p_path)
                            if Path(p_str).name.startswith("launch_params_"):
                                loaded = yaml.unsafe_load(Path(p_str).read_text(encoding="utf-8"))
                                if isinstance(loaded, dict):
                                    # Extract inner ros__parameters regardless of /** or node key
                                    for v in loaded.values():
                                        if isinstance(v, dict) and "ros__parameters" in v:
                                            param_dicts.append(v["ros__parameters"])
                            else:
                                param_files.append(p_str)
                        nodes.append(
                            {
                                "param_files": param_files,
                                "param_dicts": param_dicts,
                                "remappings": list(n.expanded_remapping_rules or []),
                                "arguments": [str(a) for a in (getattr(n, "_Node__arguments", None) or [])],
                            }
                        )
    return nodes, ctx


def test_native_launches_sim_flag_and_physical_defaults() -> None:
    # 1. game_controller/launch/launch.py
    gc_launch = _WS_SRC / "game_controller" / "launch" / "launch.py"
    gc_phys, _ = _eval_launch_node(gc_launch, {"sim": "false"})
    assert gc_phys[0]["param_dicts"][0]["enable_ip_white_list"] is True
    assert list(gc_phys[0]["param_dicts"][0]["ip_white_list"]) == ["172.169.80.24"]

    gc_sim, _ = _eval_launch_node(gc_launch, {"sim": "true"})
    assert gc_sim[0]["param_dicts"][0]["enable_ip_white_list"] is False
    assert list(gc_sim[0]["param_dicts"][0]["ip_white_list"]) == ["127.0.0.1"]

    # 2. vision/launch/launch.py
    vis_launch = _WS_SRC / "vision" / "launch" / "launch.py"
    vis_phys, _ = _eval_launch_node(vis_launch, {"sim": "false"})
    assert not vis_phys[0]["remappings"]

    vis_sim, _ = _eval_launch_node(
        vis_launch, {"sim": "true", "robot_name": "blue_1", "robot_type": "t1"}
    )
    vis_params = vis_sim[0]["param_dicts"][0]
    assert vis_params["detection_model_path"] == "model/sim_data_det_0126.onnx"
    assert vis_params["segmentation_model_path"] == "model/sim_data_seg_0126.onnx"
    assert vis_params["use_sim_time"] is True
    assert vis_params["camera_info_topic"] == "/blue_1/camera/camera_info"
    assert ("/boostercamera/head/rgb/camera_info", "/blue_1/camera/camera_info") in vis_sim[0]["remappings"]
    assert vis_sim[0]["arguments"][1].endswith("vision_t1.yaml")

    # 3. brain/launch/launch.py
    brain_launch = _WS_SRC / "brain" / "launch" / "launch.py"
    brain_sim, _ = _eval_launch_node(
        brain_launch,
        {
            "sim": "true",
            "robot_type": "t1",
            "role": "striker",
            "team_id": "70",
            "player_id": "1",
            "number_of_players": "1",
            "game_control_ip": "127.0.0.1",
        },
    )
    assert any(p.endswith("brain_t1.yaml") for p in brain_sim[0]["param_files"])
    override_dict = brain_sim[0]["param_dicts"][-1]
    assert override_dict["use_sim_time"] is True
    assert override_dict["game.player_role"] == "striker"
    assert override_dict["game.team_id"] == 70
    assert override_dict["game.player_id"] == 1
    assert override_dict["game.number_of_players"] == 1
    assert override_dict["game_control_ip"] == "127.0.0.1"
    assert override_dict["vision_config_local_path"].endswith("vision_t1.yaml")
    assert override_dict["locator.use_mujoco_localization"] is False

    brain_mujoco, _ = _eval_launch_node(
        brain_launch,
        {
            "sim": "true",
            "robot_name": "red_2",
            "robot_type": "t1",
            "use_mujoco_localization": "true",
        },
    )
    mj_override = brain_mujoco[0]["param_dicts"][-1]
    assert mj_override["locator.use_mujoco_localization"] is True
    assert mj_override["locator.mujoco_localization_topic"] == "/red_2/ground_truth/localization"

    demo_launch = _PKG_ROOT / "launch" / "robocup_demo_robot.launch.py"
    demo_nodes, _ = _eval_launch_node(
        demo_launch,
        {
            "robot_name": "blue_2",
            "enable_brain": "true",
            "use_mujoco_localization": "true",
        },
    )
    brain_demo_params = demo_nodes[-1]["param_dicts"][-1]
    assert brain_demo_params["locator.use_mujoco_localization"] is True
    assert brain_demo_params["locator.mujoco_localization_topic"] == "/blue_2/ground_truth/localization"


def test_match_supervisor_roster_sides_and_sidelines() -> None:
    import importlib.util
    import math
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
    from pumasim_booster_bridge.match_supervisor import (
        compute_sideline_pose,
        parse_roster,
        resolve_team_defends_positive_x,
    )

    roster = parse_roster(
        robots_csv="blue_1,blue_2,red_1:71:1:goal_keeper:left:t1,red_2:71:2:striker:right:k1"
    )
    assert [(r.robot_name, r.team_id, r.player_id, r.player_role, r.start_pos, r.robot_type) for r in roster] == [
        ("blue_1", 70, 1, "goal_keeper", "left", "t1"),
        ("blue_2", 70, 2, "striker", "right", "t1"),
        ("red_1", 71, 1, "goal_keeper", "left", "t1"),
        ("red_2", 71, 2, "striker", "right", "k1"),
    ]

    assert resolve_team_defends_positive_x(70, (70, 71), 1) is False
    assert resolve_team_defends_positive_x(71, (70, 71), 1) is True
    # Swapping teams[] slots switches sides automatically
    assert resolve_team_defends_positive_x(70, (71, 70), 0) is True
    assert resolve_team_defends_positive_x(71, (71, 70), 0) is False
    # Optional legacy mode where teams[] stay fixed and only first_half=0 flips
    assert resolve_team_defends_positive_x(70, (70, 71), 0, second_half_swaps_teams=True) is True

    bx, by, byaw = compute_sideline_pose(roster[0], defends_positive_x=False)
    assert bx < -4.0 and by > 4.5 and byaw == pytest.approx(-math.pi / 2.0)
    rx, ry, ryaw = compute_sideline_pose(roster[2], defends_positive_x=True)
    assert rx > 4.0 and ry < -4.5 and ryaw == pytest.approx(math.pi / 2.0)

    # Evaluate robocup_demo_match.launch.py structure
    match_launch = _PKG_ROOT / "launch" / "robocup_demo_match.launch.py"
    spec = importlib.util.spec_from_file_location(match_launch.stem, match_launch)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    ctx = LaunchContext()
    emitted = []
    for e in ld.entities:
        if isinstance(e, DeclareLaunchArgument):
            e.visit(ctx)
        elif isinstance(e, OpaqueFunction):
            ctx.launch_configurations.update({
                "robots": "blue_1,blue_2,red_1:71:1:striker:left:t1",
                "robot_type": "k1",
            })
            emitted = e.visit(ctx) or []
    includes = [a for a in emitted if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 4  # 1 multi bridge + 3 robot stacks
    assert dict(includes[0].launch_arguments)["robot_type"] == "auto"
    robot_launches = [dict(include.launch_arguments) for include in includes[1:]]
    assert [args["robot_type"] for args in robot_launches] == ["k1", "k1", "t1"]
    assert all(args["enable_bridge"] == "false" for args in robot_launches)


@pytest.mark.skipif(not _RUN_E2E, reason="opt-in real separate-process ROS 2 test")
def test_multi_brain_udp_reuseport_and_gamecontrol_side_switch_e2e(
    monkeypatch, tmp_path: Path
) -> None:
    import math
    import rclpy
    from booster_interface.msg import Odometer
    from game_controller_interface.msg import GameControlData, RobotInfo, TeamInfo
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

    env = _local_ros_env(monkeypatch, str(166 + (os.getpid() % 12)))
    sim_log = tmp_path / "sim.log"
    sup_log = tmp_path / "sup.log"
    bridge_log = tmp_path / "bridge.log"
    b1_log = tmp_path / "b1.log"
    b2_log = tmp_path / "b2.log"
    fhs = [open(p, "w", encoding="utf-8") for p in (sim_log, sup_log, bridge_log, b1_log, b2_log)]
    sim_fh, sup_fh, bridge_fh, b1_fh, b2_fh = fhs

    sim = subprocess.Popen(
        [
            "uv", "run", "--locked", "--no-sync", "--project", str(_PUMASIM_ROOT),
            "pumanoids-sim", "run", "--config", "single_t1", "--headless", "--ros2", "--duration", "35",
        ],
        cwd=str(_PUMASIM_ROOT), env=env, stdout=sim_fh, stderr=subprocess.STDOUT, start_new_session=True,
    )
    bridge = subprocess.Popen(
        [
            "ros2", "launch", "pumasim_booster_bridge", "booster_bridge.launch.py",
            "robots:=blue_1", "robot_type:=t1", "layout:=root", "relay_rgb:=false",
        ],
        env=env, stdout=bridge_fh, stderr=subprocess.STDOUT, start_new_session=True,
    )
    sup = subprocess.Popen(
        [
            "ros2", "run", "pumasim_booster_bridge", "match_supervisor", "--ros-args",
            "-p", "robots_csv:=blue_1:70:1:goal_keeper:left:t1",
        ],
        env=env, stdout=sup_fh, stderr=subprocess.STDOUT, start_new_session=True,
    )
    b1 = subprocess.Popen(
        [
            "ros2", "launch", "pumasim_booster_bridge", "robocup_demo_robot.launch.py",
            "robot_name:=blue_1", "namespaced:=true", "enable_vision:=false", "enable_brain:=true",
            "enable_com:=true", "team_id:=70", "player_id:=1", "player_role:=goal_keeper",
            "player_start_pos:=left", "number_of_players:=2",
        ],
        env=env, stdout=b1_fh, stderr=subprocess.STDOUT, start_new_session=True,
    )
    b2 = subprocess.Popen(
        [
            "ros2", "launch", "pumasim_booster_bridge", "robocup_demo_robot.launch.py",
            "robot_name:=blue_2", "namespaced:=true", "enable_vision:=false", "enable_brain:=true",
            "enable_com:=true", "team_id:=70", "player_id:=2", "player_role:=striker",
            "player_start_pos:=right", "number_of_players:=2",
        ],
        env=env, stdout=b2_fh, stderr=subprocess.STDOUT, start_new_session=True,
    )

    rclpy.init()
    obs = rclpy.create_node("multi_brain_gc_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(obs)
    gc_pub = obs.create_publisher(GameControlData, "/robocup/game_controller", 10)
    locs: list[tuple[float, float, float]] = []
    world_locs: list[tuple[float, float]] = []
    rel_10 = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST, depth=10, reliability=QoSReliabilityPolicy.RELIABLE
    )
    from nav_msgs.msg import Odometry
    obs.create_subscription(
        Odometer,
        "/blue_1/ground_truth/localization",
        lambda m: locs.append((m.x, m.y, m.theta)),
        rel_10,
    )
    obs.create_subscription(
        Odometry,
        "/blue_1/ground_truth/odom",
        lambda m: world_locs.append((m.pose.pose.position.x, m.pose.pose.position.y)),
        rel_10,
    )

    def _make_gc(slot0_team: int, slot1_team: int, first_half: int, state: int) -> GameControlData:
        m = GameControlData()
        m.version = 20
        m.players_per_team = 11
        m.state = state
        m.first_half = first_half
        m.kick_off_team = slot0_team
        m.teams = [
            TeamInfo(team_number=slot0_team, goalkeeper=1, players=[RobotInfo() for _ in range(11)]),
            TeamInfo(team_number=slot1_team, goalkeeper=1, players=[RobotInfo() for _ in range(11)]),
        ]
        return m

    try:
        # 1) Verify both same-team brains bound UDP 10070 via SO_REUSEPORT without bind failed
        def _both_udp_bound() -> bool:
            executor.spin_once(timeout_sec=0.05)
            t1 = b1_log.read_text(encoding="utf-8", errors="ignore")
            t2 = b2_log.read_text(encoding="utf-8", errors="ignore")
            assert "bind failed" not in t1 and "bind failed" not in t2
            assert "team broadcast failed" not in t1 and "team broadcast failed" not in t2
            return (
                "Team communication broadcast/listen on UDP port 10070" in t1
                and "Team communication broadcast/listen on UDP port 10070" in t2
            )

        _poll_until(_both_udp_bound, timeout=20.0, step=0.1)

        # 2) GameController INITIAL with team 70 on slot 0 (left half, -X) -> places blue_1 on -X sideline
        def _placed_left_half() -> bool:
            gc_pub.publish(_make_gc(70, 71, 1, 0))
            executor.spin_once(timeout_sec=0.05)
            return (
                bool(world_locs) and world_locs[-1][0] < -4.0 and world_locs[-1][1] > 4.5
                and bool(locs) and locs[-1][0] < -4.0 and locs[-1][1] > 4.5
            )

        _poll_until(_placed_left_half, timeout=15.0, step=0.1)

        # 3) GameController switches team 70 to slot 1 (right half, +X) -> PumaSim places blue_1 on +X
        #    half's own-left sideline (world x > +4, y < -4.5) AND flips team_odom 180° so team_loc stays x < -4, y > 4.5!
        locs.clear()
        world_locs.clear()
        def _switched_half_and_flipped() -> bool:
            gc_pub.publish(_make_gc(71, 70, 0, 0))
            executor.spin_once(timeout_sec=0.05)
            return (
                bool(world_locs) and world_locs[-1][0] > 4.0 and world_locs[-1][1] < -4.5
                and bool(locs) and locs[-1][0] < -4.0 and locs[-1][1] > 4.5 and locs[-1][2] < -1.0
            )

        _poll_until(_switched_half_and_flipped, timeout=15.0, step=0.1)
    finally:
        obs.destroy_node()
        rclpy.shutdown()
        for proc in (b1, b2, sup, bridge, sim):
            _terminate_tree(proc)
        for fh in fhs:
            fh.close()
