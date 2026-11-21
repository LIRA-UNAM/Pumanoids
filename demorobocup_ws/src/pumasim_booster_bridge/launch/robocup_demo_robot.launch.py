"""Launch one robot's Booster translation bridge plus optional Pumanoids vision/brain nodes."""

import os
from pathlib import Path

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pumasim_booster_bridge.launch_utils import load_node_parameters_from_yaml


def _as_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise RuntimeError(
        f"Launch argument '{name}' must be 'true' or 'false', got {value!r}"
    )


def _prefix(namespace: str, topic: str) -> str:
    clean_ns = namespace.strip().rstrip("/")
    clean_topic = topic.lstrip("/")
    return f"{clean_ns}/{clean_topic}" if clean_ns else f"/{clean_topic}"


def _resolve_package_dir(package_name: str, fallback_src_subpath: Path | None) -> str:
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        if fallback_src_subpath is not None and fallback_src_subpath.is_dir():
            return str(fallback_src_subpath)
        raise


def _find_ws_src(start: Path) -> Path | None:
    for parent in start.parents:
        if (parent / "src" / "pumasim_booster_bridge").is_dir():
            return parent / "src"
        if (parent / "pumasim_booster_bridge").is_dir() and parent.name == "src":
            return parent
    return None


def build_demo_remappings(
    *,
    robot_name: str,
    booster_namespace: str,
    rgb_target: str,
    camera_info_source: str,
    depth_target: str,
    isolate_tf: bool,
) -> list[tuple[str, str]]:
    """Build deterministic topic & TF remappings for one robot's demo consumers.

    - `/robocup/game_controller` is intentionally NOT remapped so one shared
      GameController feeds all robots.
    - When `isolate_tf` is True (multi-robot), `brain`'s fixed `odom -> base_link`
      broadcasts are isolated onto `/<robot>/booster/tf` (and `/tf_static`),
      preserving global `/tf` exclusively for PumaSim's namespaced `<robot>/*` TF.
    """

    remappings = [
        ("/boostercamera/head/rgb", rgb_target),
        ("/boostercamera/head/rgb/camera_info", camera_info_source),
        ("/boostercamera/head/depth", depth_target),
        ("/low_state", _prefix(booster_namespace, "low_state")),
        ("/odometer_state", _prefix(booster_namespace, "odometer_state")),
        ("/head_pose", _prefix(booster_namespace, "head_pose")),
        (
            "fall_down_recovery_state",
            _prefix(booster_namespace, "fall_down_recovery_state"),
        ),
        (
            "/fall_down_recovery_state",
            _prefix(booster_namespace, "fall_down_recovery_state"),
        ),
        ("LocoApiTopicReq", _prefix(booster_namespace, "LocoApiTopicReq")),
        ("/LocoApiTopicReq", _prefix(booster_namespace, "LocoApiTopicReq")),
        ("LocoApiTopicResp", _prefix(booster_namespace, "LocoApiTopicResp")),
        ("/LocoApiTopicResp", _prefix(booster_namespace, "LocoApiTopicResp")),
        ("/kick_ball", _prefix(booster_namespace, "kick_ball")),
        (
            "/booster_vision/detection",
            _prefix(booster_namespace, "booster_vision/detection"),
        ),
        (
            "/booster_vision/line_segments",
            _prefix(booster_namespace, "booster_vision/line_segments"),
        ),
        ("/booster_vision/ball", _prefix(booster_namespace, "booster_vision/ball")),
        (
            "/booster_vision/t_head2base",
            _prefix(booster_namespace, "booster_vision/t_head2base"),
        ),
        (
            "/booster_vision/cal_param",
            _prefix(booster_namespace, "booster_vision/cal_param"),
        ),
        (
            "/remote_controller_state",
            _prefix(booster_namespace, "remote_controller_state"),
        ),
        ("/play_sound", _prefix(booster_namespace, "play_sound")),
        ("/speak", _prefix(booster_namespace, "speak")),
    ]
    if isolate_tf:
        tf_topic = _prefix(booster_namespace, "tf")
        tf_static_topic = _prefix(booster_namespace, "tf_static")
        remappings.extend(
            [
                ("/tf", tf_topic),
                ("tf", tf_topic),
                ("/tf_static", tf_static_topic),
                ("tf_static", tf_static_topic),
            ]
        )
    return remappings


def _launch_robot_stack(context):
    robot_name = LaunchConfiguration("robot_name").perform(context).strip("/")
    robot_type = LaunchConfiguration("robot_type").perform(context).strip().lower()
    if robot_type not in {"t1", "k1"}:
        raise RuntimeError(f"robot_type must be 't1' or 'k1', got {robot_type!r}")

    namespaced = _as_bool(
        LaunchConfiguration("namespaced").perform(context), "namespaced"
    )
    booster_ns = f"/{robot_name}/booster" if namespaced else ""

    relay_rgb = _as_bool(
        LaunchConfiguration("relay_rgb").perform(context), "relay_rgb"
    )
    relay_depth = _as_bool(
        LaunchConfiguration("relay_depth").perform(context), "relay_depth"
    )
    relay_camera_info = _as_bool(
        LaunchConfiguration("relay_camera_info").perform(context), "relay_camera_info"
    )
    enable_vision = _as_bool(
        LaunchConfiguration("enable_vision").perform(context), "enable_vision"
    )
    enable_brain = _as_bool(
        LaunchConfiguration("enable_brain").perform(context), "enable_brain"
    )
    enable_com = _as_bool(
        LaunchConfiguration("enable_com").perform(context), "enable_com"
    )
    team_id = int(LaunchConfiguration("team_id").perform(context))
    player_id = int(LaunchConfiguration("player_id").perform(context))
    player_role = LaunchConfiguration("player_role").perform(context)
    tree = LaunchConfiguration("tree").perform(context)
    det_model = LaunchConfiguration("detection_model_path").perform(context).strip()
    seg_model = LaunchConfiguration("segmentation_model_path").perform(context).strip()

    bridge_share = get_package_share_directory("pumasim_booster_bridge")
    vision_overlay = os.path.join(bridge_share, "config", f"vision_{robot_type}.yaml")
    brain_overlay = os.path.join(bridge_share, "config", f"brain_{robot_type}.yaml")

    ws_src = _find_ws_src(Path(__file__).resolve())
    vision_cfg_override = LaunchConfiguration("vision_config_file").perform(context).strip()
    brain_cfg_override = LaunchConfiguration("brain_config_file").perform(context).strip()

    def _resolve_vision_base() -> str:
        if vision_cfg_override:
            return vision_cfg_override
        fallback = (ws_src / "vision") if ws_src is not None else None
        vision_share = _resolve_package_dir("vision", fallback)
        return os.path.join(vision_share, "config", "vision.yaml")

    camera_info_source = (
        _prefix(booster_ns, "boostercamera/head/rgb/camera_info")
        if relay_camera_info
        else f"/{robot_name}/camera/camera_info"
    )
    rgb_target = (
        _prefix(booster_ns, "boostercamera/head/rgb")
        if relay_rgb
        else f"/{robot_name}/camera/image_raw"
    )
    depth_target = (
        _prefix(booster_ns, "boostercamera/head/depth")
        if relay_depth
        else f"/{robot_name}/camera/depth/image_raw"
    )

    actions = [
        Node(
            package="pumasim_booster_bridge",
            executable="booster_bridge",
            name=f"pumasim_booster_bridge_{robot_name}",
            output="screen",
            parameters=[
                {
                    "robot_name": robot_name,
                    "robot_type": robot_type,
                    "booster_namespace": booster_ns,
                    "localization_topic": f"/{robot_name}/ground_truth/localization",
                    "relay_rgb": relay_rgb,
                    "relay_depth": relay_depth,
                    "relay_camera_info": relay_camera_info,
                }
            ],
        )
    ]

    remappings = build_demo_remappings(
        robot_name=robot_name,
        booster_namespace=booster_ns,
        rgb_target=rgb_target,
        camera_info_source=camera_info_source,
        depth_target=depth_target,
        isolate_tf=namespaced,
    )

    if enable_vision:
        base_vision = _resolve_vision_base()
        vision_params = {
            "use_sim_time": True,
            "offline_mode": False,
            "camera_type": "realsense",
            "camera_info_topic": camera_info_source,
            "save_data": False,
        }
        if det_model:
            vision_params["detection_model_path"] = det_model
        if seg_model:
            vision_params["segmentation_model_path"] = seg_model
        actions.append(
            Node(
                package="vision",
                executable="vision_node",
                name=f"vision_{robot_name}",
                output="screen",
                arguments=[base_vision, vision_overlay],
                parameters=[vision_params],
                remappings=remappings,
            )
        )

    if enable_brain:
        brain_share = _resolve_package_dir(
            "brain", (ws_src / "brain") if ws_src is not None else None
        )
        base_brain = brain_cfg_override or os.path.join(brain_share, "config", "config.yaml")
        base_vision = _resolve_vision_base()
        tree_file = tree if tree.endswith(".xml") else f"{tree}.xml"
        tree_path = os.path.join(brain_share, "behavior_trees", tree_file)

        # Load base YAML -> T1/K1 overlay -> instance overrides explicitly so
        # `brain_<robot>` receives `brain_node:` parameters in the exact order.
        base_brain_params = load_node_parameters_from_yaml(base_brain)
        overlay_brain_params = load_node_parameters_from_yaml(brain_overlay)
        instance_brain_params = {
            "use_sim_time": True,
            "tree_file_path": tree_path,
            "vision_config_path": base_vision,
            "vision_config_local_path": vision_overlay,
            "game.team_id": team_id,
            "game.player_id": player_id,
            "game.player_role": player_role,
            "enable_com": enable_com,
            "rerunLog.enable_file": False,
            "rerunLog.enable_tcp": False,
            "vision.image_topic": rgb_target,
            "vision.depth_image_topic": depth_target,
        }

        actions.append(
            Node(
                package="brain",
                executable="brain_node",
                name=f"brain_{robot_name}",
                output="screen",
                parameters=[
                    base_brain_params,
                    overlay_brain_params,
                    instance_brain_params,
                ],
                remappings=remappings,
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="blue_1"),
            DeclareLaunchArgument("robot_type", default_value="t1"),
            DeclareLaunchArgument(
                "namespaced",
                default_value="false",
                description="True to isolate Booster/demo topics and TF under /<robot_name>/booster for multi-robot runs.",
            ),
            DeclareLaunchArgument("relay_rgb", default_value="true"),
            DeclareLaunchArgument("relay_depth", default_value="false"),
            DeclareLaunchArgument("relay_camera_info", default_value="false"),
            DeclareLaunchArgument("enable_vision", default_value="false"),
            DeclareLaunchArgument("enable_brain", default_value="false"),
            DeclareLaunchArgument("enable_com", default_value="false"),
            DeclareLaunchArgument("team_id", default_value="70"),
            DeclareLaunchArgument("player_id", default_value="1"),
            DeclareLaunchArgument("player_role", default_value="goal_keeper"),
            DeclareLaunchArgument("tree", default_value="game.xml"),
            DeclareLaunchArgument("brain_config_file", default_value=""),
            DeclareLaunchArgument("vision_config_file", default_value=""),
            DeclareLaunchArgument("detection_model_path", default_value=""),
            DeclareLaunchArgument("segmentation_model_path", default_value=""),
            OpaqueFunction(function=_launch_robot_stack),
        ]
    )
