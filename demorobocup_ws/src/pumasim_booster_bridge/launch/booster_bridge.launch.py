"""Launch one or more Booster protocol bridge nodes over PumaSim generic topics."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    raise RuntimeError(f"Launch argument '{name}' must be 'true' or 'false', got {value!r}")


def _resolve_namespace(robot_name: str, *, robot_count: int, layout: str) -> str:
    if layout == "root":
        if robot_count != 1:
            raise RuntimeError("layout:='root' is valid only when launching one robot")
        return ""
    if layout == "namespaced":
        return f"/{robot_name}/booster"
    if layout == "auto":
        return "" if robot_count == 1 else f"/{robot_name}/booster"
    raise RuntimeError(f"Unknown layout {layout!r}; expected auto, root, or namespaced")


def _launch_bridges(context):
    raw_robots = LaunchConfiguration("robots").perform(context)
    robots = [r.strip().strip("/") for r in raw_robots.split(",") if r.strip()]
    if not robots:
        raise RuntimeError("Launch argument 'robots' must name at least one robot")

    robot_type = LaunchConfiguration("robot_type").perform(context)
    layout = LaunchConfiguration("layout").perform(context).strip().lower()
    single_process = _as_bool(
        LaunchConfiguration("single_process").perform(context), "single_process"
    )
    relay_rgb = _as_bool(LaunchConfiguration("relay_rgb").perform(context), "relay_rgb")
    relay_depth = _as_bool(
        LaunchConfiguration("relay_depth").perform(context), "relay_depth"
    )
    relay_camera_info = _as_bool(
        LaunchConfiguration("relay_camera_info").perform(context), "relay_camera_info"
    )

    if single_process and len(robots) > 1:
        if layout == "root":
            raise RuntimeError("layout:='root' is valid only when launching one robot")
        return [
            Node(
                package="pumasim_booster_bridge",
                executable="booster_bridge",
                name="pumasim_booster_bridge_multi",
                output="screen",
                parameters=[
                    {
                        "robot_names_csv": ",".join(robots),
                        "robot_type": robot_type,
                        "layout": layout,
                        "relay_rgb": relay_rgb,
                        "relay_depth": relay_depth,
                        "relay_camera_info": relay_camera_info,
                    }
                ],
            )
        ]

    actions = []
    for robot_name in robots:
        booster_ns = _resolve_namespace(
            robot_name, robot_count=len(robots), layout=layout
        )
        actions.append(
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
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robots",
                default_value="blue_1",
                description="Comma-separated PumaSim robot names (e.g. blue_1 or blue_1,red_1).",
            ),
            DeclareLaunchArgument(
                "robot_type",
                default_value="auto",
                description="Robot joint profile: auto | t1 | k1.",
            ),
            DeclareLaunchArgument(
                "layout",
                default_value="auto",
                description="Booster topic layout: auto (root for 1 robot, /<robot>/booster for 2+), root, or namespaced.",
            ),
            DeclareLaunchArgument(
                "single_process",
                default_value="false",
                description="True to host all robot endpoints inside one booster_bridge process (lower DDS overhead in 5v5).",
            ),
            DeclareLaunchArgument(
                "relay_rgb",
                default_value="true",
                description="Relay BEST_EFFORT /<robot>/camera/image_raw to RELIABLE boostercamera/head/rgb for vision/brain.",
            ),
            DeclareLaunchArgument(
                "relay_depth",
                default_value="false",
                description="Relay /<robot>/camera/depth/image_raw to boostercamera/head/depth when enabled.",
            ),
            DeclareLaunchArgument(
                "relay_camera_info",
                default_value="false",
                description="Keep false to remap /<robot>/camera/camera_info directly (SensorDataQoS).",
            ),
            OpaqueFunction(function=_launch_bridges),
        ]
    )
