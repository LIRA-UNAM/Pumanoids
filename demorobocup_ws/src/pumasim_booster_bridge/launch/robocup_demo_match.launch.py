"""Launch a multi-robot RoboCup demo match (1 bridge + 1 GC + match supervisor + N vision/brain)."""

from __future__ import annotations

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pumasim_booster_bridge.match_supervisor import parse_roster


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _launch_match(context):
    robots_csv = LaunchConfiguration("robots").perform(context).strip()
    roster_file = LaunchConfiguration("roster_file").perform(context).strip()
    robot_type_default = LaunchConfiguration("robot_type").perform(context).strip().lower()
    roster = parse_roster(
        robots_csv=robots_csv,
        roster_file=roster_file,
        default_robot_type="k1" if robot_type_default == "k1" else "t1",
    )
    if not roster:
        raise RuntimeError(
            "robocup_demo_match.launch.py requires non-empty 'robots' or 'roster_file'"
        )

    enable_vision = LaunchConfiguration("enable_vision").perform(context).strip()
    enable_brain = LaunchConfiguration("enable_brain").perform(context).strip()
    enable_com = LaunchConfiguration("enable_com").perform(context).strip()
    enable_gc = _as_bool(LaunchConfiguration("enable_gc").perform(context))
    enable_supervisor = _as_bool(
        LaunchConfiguration("enable_match_supervisor").perform(context)
    )
    use_mujoco_loc = LaunchConfiguration("use_mujoco_localization").perform(context).strip()
    gc_ip = LaunchConfiguration("game_control_ip").perform(context).strip()
    gc_port = int(LaunchConfiguration("gc_port").perform(context))
    relay_rgb = LaunchConfiguration("relay_rgb").perform(context).strip()
    relay_camera_info = LaunchConfiguration("relay_camera_info").perform(context).strip()
    second_half_swaps = _as_bool(
        LaunchConfiguration("second_half_swaps_teams").perform(context)
    )
    auto_place_initial = _as_bool(
        LaunchConfiguration("auto_place_on_initial").perform(context)
    )
    auto_place_penalty = _as_bool(
        LaunchConfiguration("auto_place_on_penalty").perform(context)
    )

    bridge_share = get_package_share_directory("pumasim_booster_bridge")
    bridge_launch_path = os.path.join(bridge_share, "launch", "booster_bridge.launch.py")
    robot_launch_path = os.path.join(bridge_share, "launch", "robocup_demo_robot.launch.py")

    team_counts: dict[int, int] = {}
    for entry in roster:
        team_counts[entry.team_id] = team_counts.get(entry.team_id, 0) + 1

    robot_types = {entry.robot_type for entry in roster}
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch_path),
            launch_arguments={
                "robots": ",".join(e.robot_name for e in roster),
                "robot_type": next(iter(robot_types)) if len(robot_types) == 1 else "auto",
                "layout": "namespaced",
                "single_process": "true",
                "relay_rgb": relay_rgb,
                "relay_camera_info": relay_camera_info,
            }.items(),
        )
    ]

    if enable_gc:
        actions.append(
            Node(
                package="game_controller",
                executable="game_controller_node",
                name="game_controller_node",
                output="screen",
                parameters=[
                    {
                        "port": gc_port,
                        "enable_ip_white_list": False,
                        "ip_white_list": ["127.0.0.1"],
                    }
                ],
            )
        )

    if enable_supervisor:
        roster_payload = json.dumps(
            [
                {
                    "robot_name": e.robot_name,
                    "team_id": e.team_id,
                    "player_id": e.player_id,
                    "role": e.player_role,
                    "pos": e.start_pos,
                    "robot_type": e.robot_type,
                }
                for e in roster
            ],
            separators=(",", ":"),
        )
        actions.append(
            Node(
                package="pumasim_booster_bridge",
                executable="match_supervisor",
                name="pumasim_match_supervisor",
                output="screen",
                parameters=[
                    {
                        "roster_json": roster_payload,
                        "second_half_swaps_teams": second_half_swaps,
                        "auto_place_on_initial": auto_place_initial,
                        "auto_place_on_penalty": auto_place_penalty,
                    }
                ],
            )
        )

    for entry in roster:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch_path),
                launch_arguments={
                    "robot_name": entry.robot_name,
                    "robot_type": entry.robot_type,
                    "namespaced": "true",
                    "enable_bridge": "false",
                    "relay_rgb": relay_rgb,
                    "relay_camera_info": relay_camera_info,
                    "enable_vision": enable_vision,
                    "enable_brain": enable_brain,
                    "enable_com": enable_com,
                    "team_id": str(entry.team_id),
                    "player_id": str(entry.player_id),
                    "player_role": entry.player_role,
                    "player_start_pos": entry.start_pos,
                    "number_of_players": str(team_counts[entry.team_id]),
                    "game_control_ip": gc_ip,
                    "use_mujoco_localization": use_mujoco_loc,
                }.items(),
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robots",
                default_value="blue_1,blue_2",
                description="CSV de robots o specs name[:team[:player[:role[:pos[:type]]]]].",
            ),
            DeclareLaunchArgument("roster_file", default_value=""),
            DeclareLaunchArgument("robot_type", default_value="auto"),
            DeclareLaunchArgument("enable_vision", default_value="true"),
            DeclareLaunchArgument("enable_brain", default_value="true"),
            DeclareLaunchArgument("enable_com", default_value="true"),
            DeclareLaunchArgument("enable_gc", default_value="true"),
            DeclareLaunchArgument("enable_match_supervisor", default_value="true"),
            DeclareLaunchArgument("use_mujoco_localization", default_value="false"),
            DeclareLaunchArgument("game_control_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("gc_port", default_value="3838"),
            DeclareLaunchArgument("relay_rgb", default_value="true"),
            DeclareLaunchArgument("relay_camera_info", default_value="false"),
            DeclareLaunchArgument("second_half_swaps_teams", default_value="false"),
            DeclareLaunchArgument("auto_place_on_initial", default_value="true"),
            DeclareLaunchArgument("auto_place_on_penalty", default_value="true"),
            OpaqueFunction(function=_launch_match),
        ]
    )
