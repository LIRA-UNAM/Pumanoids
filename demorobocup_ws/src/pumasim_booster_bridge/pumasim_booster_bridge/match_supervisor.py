"""Match-level supervisor translating GameController state to PumaSim poses & field flips."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from game_controller_interface.msg import GameControlData
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool
import yaml


def _wrap_angle(angle: float) -> float:
    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    if wrapped <= -math.pi:
        return math.pi
    return wrapped


@dataclass(frozen=True)
class RobotRosterEntry:
    robot_name: str
    team_id: int
    player_id: int
    player_role: str = "striker"
    start_pos: str = "left"
    robot_type: str = "t1"


def _infer_team_and_player(robot_name: str, index: int) -> tuple[int, int]:
    lower = robot_name.lower()
    team_id = 71 if lower.startswith("red") else 70
    player_id = index + 1
    parts = lower.rsplit("_", 1)
    if len(parts) == 2 and parts[1].isdigit():
        player_id = int(parts[1])
    return team_id, player_id


def parse_roster(
    *,
    robots_csv: str = "",
    roster_file: str = "",
    roster_json: str = "",
    default_robot_type: str = "t1",
) -> list[RobotRosterEntry]:
    """Parse team roster from YAML file, JSON string, or CLI CSV spec.

    CSV item grammar: ``name[:team_id[:player_id[:role[:pos[:robot_type]]]]]``
    """

    entries: list[RobotRosterEntry] = []
    raw_items: list[dict[str, Any]] = []

    if roster_file.strip():
        loaded = yaml.safe_load(Path(roster_file).read_text(encoding="utf-8")) or {}
        if isinstance(loaded, dict):
            raw_items = list(loaded.get("robots") or [])
        elif isinstance(loaded, list):
            raw_items = list(loaded)
    elif roster_json.strip():
        parsed = json.loads(roster_json)
        if isinstance(parsed, list):
            raw_items = parsed

    if raw_items:
        for idx, item in enumerate(raw_items):
            name = str(item.get("robot_name") or item.get("name") or f"blue_{idx + 1}").strip("/")
            default_team, default_pid = _infer_team_and_player(name, idx)
            pid = int(item.get("player_id", default_pid))
            default_role = "goal_keeper" if pid == 1 else "striker"
            default_pos = "left" if pid % 2 == 1 else "right"
            entries.append(
                RobotRosterEntry(
                    robot_name=name,
                    team_id=int(item.get("team_id", default_team)),
                    player_id=pid,
                    player_role=str(item.get("role", item.get("player_role", default_role))),
                    start_pos=str(item.get("pos", item.get("start_pos", default_pos))).lower(),
                    robot_type=str(item.get("robot_type", default_robot_type)).lower(),
                )
            )
        return entries

    for idx, token in enumerate(r.strip() for r in robots_csv.split(",") if r.strip()):
        parts = token.split(":")
        name = parts[0].strip("/")
        default_team, default_pid = _infer_team_and_player(name, idx)
        team_id = int(parts[1]) if len(parts) > 1 and parts[1] else default_team
        pid = int(parts[2]) if len(parts) > 2 and parts[2] else default_pid
        default_role = "goal_keeper" if pid == 1 else "striker"
        default_pos = "left" if pid % 2 == 1 else "right"
        role = parts[3] if len(parts) > 3 and parts[3] else default_role
        pos = (parts[4] if len(parts) > 4 and parts[4] else default_pos).lower()
        rtype = (parts[5] if len(parts) > 5 and parts[5] else default_robot_type).lower()
        entries.append(
            RobotRosterEntry(
                robot_name=name,
                team_id=team_id,
                player_id=pid,
                player_role=role,
                start_pos=pos,
                robot_type=rtype,
            )
        )
    return entries


def resolve_team_defends_positive_x(
    team_id: int,
    teams_order: tuple[int, int],
    first_half: int,
    *,
    second_half_swaps_teams: bool = False,
) -> bool | None:
    """Return True when ``team_id`` defends +X (attacks -X -> needs 180° flip).

    In RoboCup GameController ``teams[0]`` occupies the left field slot (-X)
    and ``teams[1]`` occupies the right field slot (+X). When the GameController
    itself swaps ``teams[0]``/``teams[1]`` on half switch, keep
    ``second_half_swaps_teams=False``; set True when ``teams`` stay fixed and
    only ``first_half`` toggles ``1 -> 0``.
    """

    left_team, right_team = teams_order
    if second_half_swaps_teams and int(first_half) == 0:
        left_team, right_team = right_team, left_team
    if team_id == left_team:
        return False
    if team_id == right_team:
        return True
    return None


def compute_sideline_pose(
    entry: RobotRosterEntry,
    *,
    defends_positive_x: bool,
    field_length: float = 14.0,
    field_width: float = 9.0,
    sideline_margin: float = 0.25,
) -> tuple[float, float, float]:
    """Compute world ``(x, y, yaw)`` on the team's assigned sideline facing inward."""

    # Spread teammates deterministically along [-0.38*L .. -0.14*L] in own half.
    slot = max(0, min(4, entry.player_id - 1))
    x_own = -0.38 * field_length + slot * (0.06 * field_length)
    y_edge = (field_width / 2.0) + sideline_margin
    if entry.start_pos == "right":
        y_own = -y_edge
        yaw_own = math.pi / 2.0
    else:
        y_own = y_edge
        yaw_own = -math.pi / 2.0

    if defends_positive_x:
        return -x_own, -y_own, _wrap_angle(yaw_own + math.pi)
    return x_own, y_own, _wrap_angle(yaw_own)


class MatchSupervisorNode(Node):
    """Synchronizes PumaSim field orientation and sideline placement with GameController."""

    def __init__(self) -> None:
        super().__init__("pumasim_match_supervisor")
        self.declare_parameter("roster_json", "")
        self.declare_parameter("roster_file", "")
        self.declare_parameter("robots_csv", "")
        self.declare_parameter("field_length", 14.0)
        self.declare_parameter("field_width", 9.0)
        self.declare_parameter("sideline_margin", 0.25)
        self.declare_parameter("second_half_swaps_teams", False)
        self.declare_parameter("auto_place_on_initial", True)
        self.declare_parameter("auto_place_on_penalty", True)

        self._roster = parse_roster(
            robots_csv=str(self.get_parameter("robots_csv").value),
            roster_file=str(self.get_parameter("roster_file").value),
            roster_json=str(self.get_parameter("roster_json").value),
        )
        self._field_length = float(self.get_parameter("field_length").value)
        self._field_width = float(self.get_parameter("field_width").value)
        self._sideline_margin = float(self.get_parameter("sideline_margin").value)
        self._second_half_swaps = bool(self.get_parameter("second_half_swaps_teams").value)
        self._auto_place_initial = bool(self.get_parameter("auto_place_on_initial").value)
        self._auto_place_penalty = bool(self.get_parameter("auto_place_on_penalty").value)

        reliable_10 = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._flip_pubs = {
            e.robot_name: self.create_publisher(Bool, f"/{e.robot_name}/set_field_flip", reliable_10)
            for e in self._roster
        }
        self._pose_pubs = {
            e.robot_name: self.create_publisher(PoseStamped, f"/{e.robot_name}/set_pose", reliable_10)
            for e in self._roster
        }
        self._last_flip: dict[str, bool] = {}
        self._placed_epoch: dict[str, tuple[Any, ...]] = {}

        self.create_subscription(
            GameControlData,
            "/robocup/game_controller",
            self._on_game_control,
            reliable_10,
        )

    def _publish_pose(self, robot_name: str, x: float, y: float, yaw: float) -> None:
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._pose_pubs[robot_name].publish(msg)

    def _on_game_control(self, msg: GameControlData) -> None:
        teams_order = (int(msg.teams[0].team_number), int(msg.teams[1].team_number))
        first_half = int(msg.first_half)
        state = int(msg.state)

        penalties_by_team: dict[int, list[int]] = {}
        for team_info in msg.teams:
            penalties_by_team[int(team_info.team_number)] = [
                int(p.penalty) for p in team_info.players
            ]

        for entry in self._roster:
            defends_pos_x = resolve_team_defends_positive_x(
                entry.team_id,
                teams_order,
                first_half,
                second_half_swaps_teams=self._second_half_swaps,
            )
            if defends_pos_x is None:
                continue

            flip_pub = self._flip_pubs[entry.robot_name]
            flip_changed = self._last_flip.get(entry.robot_name) != defends_pos_x
            if flip_changed:
                flip_msg = Bool()
                flip_msg.data = defends_pos_x
                flip_pub.publish(flip_msg)
                if flip_pub.get_subscription_count() > 0:
                    self._last_flip[entry.robot_name] = defends_pos_x

            team_penalties = penalties_by_team.get(entry.team_id, [])
            p_idx = entry.player_id - 1
            cur_penalty = team_penalties[p_idx] if 0 <= p_idx < len(team_penalties) else 0

            target_epoch: tuple[Any, ...] | None = self._placed_epoch.get(entry.robot_name)
            if self._auto_place_penalty and cur_penalty != 0:
                target_epoch = ("penalty", defends_pos_x, first_half)
            elif self._auto_place_initial and state == 0:
                target_epoch = ("initial", defends_pos_x, first_half)
            elif self._auto_place_initial and flip_changed:
                target_epoch = ("flip", defends_pos_x, first_half)

            if target_epoch is not None and self._placed_epoch.get(entry.robot_name) != target_epoch:
                x, y, yaw = compute_sideline_pose(
                    entry,
                    defends_positive_x=defends_pos_x,
                    field_length=self._field_length,
                    field_width=self._field_width,
                    sideline_margin=self._sideline_margin,
                )
                self._publish_pose(entry.robot_name, x, y, yaw)
                if self._pose_pubs[entry.robot_name].get_subscription_count() > 0:
                    self._placed_epoch[entry.robot_name] = target_epoch


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MatchSupervisorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
