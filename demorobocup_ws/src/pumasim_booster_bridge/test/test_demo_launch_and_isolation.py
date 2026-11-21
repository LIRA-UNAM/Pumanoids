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
    res = subprocess.run(
        ["ros2", "param", "get", "--no-daemon", node_name, param_name],
        env=env,
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    return res.stdout.strip() if res.returncode == 0 else ""


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
