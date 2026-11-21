"""Pure-Python Booster RoboCup protocol & state translation over PumaSim generic signals."""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any

# Booster LocoApi IDs from booster_robotics_sdk/include/booster/robot/b1/b1_loco_api.hpp
# and Pumanoids demorobocup_ws/src/brain/src/robot_client.cpp.
CHANGE_MODE_API = 2000
MOVE_API = 2001
ROTATE_HEAD_API = 2004
WAVE_HAND_API = 2005
ROTATE_HEAD_WITH_DIRECTION_API = 2006
LIE_DOWN_API = 2007
GET_UP_API = 2008
GET_FRAME_TRANSFORM_API = 2011
GET_MODE_API = 2017
GET_STATUS_API = 2018
GET_ROBOT_INFO_API = 2022
SHOOT_API = 2024
GET_UP_WITH_MODE_API = 2025
RESET_ODOMETRY_API = 2031
VISUAL_KICK_API = 2038
RL_KICK_BALL_API = 100011
RL_FANCY_KICK_BALL_API = 100012

# RobotMode enum values
UNKNOWN_MODE = -1
DAMPING_MODE = 0
PREPARE_MODE = 1
WALKING_MODE = 2
CUSTOM_MODE = 3
SOCCER_MODE = 4
_VALID_MODES = {DAMPING_MODE, PREPARE_MODE, WALKING_MODE, CUSTOM_MODE, SOCCER_MODE}
_LOCOMOTION_MODES = {WALKING_MODE, SOCCER_MODE}

_BODY_CONTROL_BY_MODE = {
    DAMPING_MODE: 1,  # kDamping
    PREPARE_MODE: 2,  # kPrepare
    WALKING_MODE: 3,  # kHumanlikeGait
    CUSTOM_MODE: 6,  # kCustom
    SOCCER_MODE: 5,  # kSoccerGait
}

FRAME_BODY = 0
FRAME_HEAD = 1
FRAME_LEFT_HAND = 2
FRAME_RIGHT_HAND = 3
FRAME_LEFT_FOOT = 4
FRAME_RIGHT_FOOT = 5

FRAME_TOPIC_SUFFIX_BY_ID: dict[int, str] = {
    FRAME_HEAD: "head_pose",
    FRAME_LEFT_HAND: "left_hand_pose",
    FRAME_RIGHT_HAND: "right_hand_pose",
    FRAME_LEFT_FOOT: "left_foot_pose",
    FRAME_RIGHT_FOOT: "right_foot_pose",
}

SKILL_NAME_BY_API: dict[int, str] = {
    WAVE_HAND_API: "wave_hand",
    LIE_DOWN_API: "lie_down",
    GET_UP_API: "get_up",
    SHOOT_API: "shoot",
    GET_UP_WITH_MODE_API: "get_up_with_mode",
    VISUAL_KICK_API: "visual_kick",
    RL_KICK_BALL_API: "kick",
    RL_FANCY_KICK_BALL_API: "fancy_kick",
}

# RPC status codes from booster_robotics_sdk/include/booster/robot/rpc/error.hpp
RPC_STATUS_INVALID = -1
RPC_STATUS_SUCCESS = 0
RPC_STATUS_TIMEOUT = 100
RPC_STATUS_BAD_REQUEST = 400
RPC_STATUS_CONFLICT = 409
RPC_STATUS_TOO_FREQUENT = 429
RPC_STATUS_INTERNAL_ERROR = 500
RPC_STATUS_SERVER_REFUSED = 501
RPC_STATUS_STATE_TRANSITION_FAILED = 502

_ROTATE_HEAD_DIRECTION_STEP_RAD = 0.05

T1_SERIAL_JOINT_ORDER = (
    "AAHead_yaw",
    "Head_pitch",
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Waist",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
)

K1_SERIAL_JOINT_ORDER = (
    "Head_Yaw",
    "Head_Pitch",
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
)

K1_PARALLEL_JOINT_ORDER = (
    "Head_Yaw",
    "Head_Pitch",
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_A",
    "Left_Ankle_B",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_A",
    "Right_Ankle_B",
)

ROBOT_PROFILES = {
    "t1": {
        "model": "T1",
        "head_joints": ("AAHead_yaw", "Head_pitch"),
        "head_limits": ((-1.57, 1.57), (-0.35, 1.22)),
        "serial_order": T1_SERIAL_JOINT_ORDER,
        "parallel_order": (),
    },
    "k1": {
        "model": "K1",
        "head_joints": ("Head_Yaw", "Head_Pitch"),
        "head_limits": ((-1.0, 1.0), (-0.349, 0.855)),
        "serial_order": K1_SERIAL_JOINT_ORDER,
        "parallel_order": K1_PARALLEL_JOINT_ORDER,
    },
}

DEFAULT_VELOCITY_LIMITS = {
    "vx": (-0.5, 1.0),
    "vy": (-0.5, 0.5),
    "wz": (-1.0, 1.0),
}


class BoosterProtocolError(ValueError):
    """Raised when an incoming Booster RPC or telemetry payload is malformed."""


@dataclass(frozen=True)
class BoosterRpcResponse:
    status: int
    body: str = ""

    @property
    def header(self) -> str:
        return json.dumps({"status": self.status}, separators=(",", ":"))


@dataclass(frozen=True)
class BoosterRpcResult:
    response: BoosterRpcResponse
    cmd_vel: tuple[float, float, float] | None = None
    joint_targets: tuple[tuple[str, ...], tuple[float, ...]] | None = None
    reset_odometry: bool = False
    skill_command: tuple[str, dict[str, Any]] | None = None


@dataclass(frozen=True)
class BoosterOdometerSample:
    x: float
    y: float
    theta: float


@dataclass(frozen=True)
class BoosterRecoverySample:
    state: int
    is_recovery_available: int
    current_planner_index: int


@dataclass(frozen=True)
class BoosterMotorSample:
    mode: int
    q: float
    dq: float
    ddq: float
    tau_est: float


@dataclass(frozen=True)
class BoosterLowStateSample:
    rpy: tuple[float, float, float]
    gyro: tuple[float, float, float]
    acc: tuple[float, float, float]
    motor_state_parallel: tuple[BoosterMotorSample, ...]
    motor_state_serial: tuple[BoosterMotorSample, ...]


@dataclass(frozen=True)
class BoosterKickReference:
    ball_x: float
    ball_y: float
    direction: float
    goal_x: float
    goal_y: float
    robot_theta_to_field: float
    power: float


def normalize_quaternion_xyzw(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("quaternion must be finite and nonzero")
    return qx / norm, qy / norm, qz / norm, qw / norm


def quaternion_xyzw_to_rpy(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float]:
    x, y, z, w = normalize_quaternion_xyzw(qx, qy, qz, qw)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_sine = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_sine)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_angle(angle: float) -> float:
    """Wrap radians into (-pi, pi]."""

    wrapped = math.atan2(math.sin(angle), math.cos(angle))
    if wrapped <= -math.pi:
        return math.pi
    return wrapped


def ground_truth_odom_to_localization(
    x: float, y: float, qx: float, qy: float, qz: float, qw: float
) -> BoosterOdometerSample:
    """Convert PumaSim world Odometry pose into Booster Odometer field coordinates."""

    yaw = quaternion_xyzw_to_rpy(qx, qy, qz, qw)[2]
    return BoosterOdometerSample(x=float(x), y=float(y), theta=wrap_angle(yaw))


def relative_odom_from_nav(
    x: float,
    y: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    origin: tuple[float, float, float],
) -> BoosterOdometerSample:
    """Express world-aligned PumaSim `/odom` pose in robot spawn/reset origin frame."""

    ox, oy, oyaw = origin
    dx = float(x) - ox
    dy = float(y) - oy
    c = math.cos(oyaw)
    s = math.sin(oyaw)
    yaw = quaternion_xyzw_to_rpy(qx, qy, qz, qw)[2]
    return BoosterOdometerSample(
        x=c * dx + s * dy,
        y=-s * dx + c * dy,
        theta=wrap_angle(yaw - oyaw),
    )


def recovery_state_from_imu(
    qx: float, qy: float, qz: float, qw: float, mode: int
) -> BoosterRecoverySample:
    """Infer upright vs fallen state from IMU gravity projection (get-up unavailable)."""

    x, y, _z, _w = normalize_quaternion_xyzw(qx, qy, qz, qw)
    # Row 2, Col 2 of R is 1 - 2*(x^2 + y^2); gravity projected in body Z is -R[2,2].
    proj_gravity_z = -1.0 + 2.0 * (x * x + y * y)
    upright = proj_gravity_z <= -0.5
    if not upright:
        return BoosterRecoverySample(state=2, is_recovery_available=0, current_planner_index=1)
    planner_index = 20 if mode == SOCCER_MODE else 8
    return BoosterRecoverySample(
        state=0, is_recovery_available=0, current_planner_index=planner_index
    )


def detect_robot_type(joint_names: Sequence[str]) -> str | None:
    name_set = set(joint_names)
    if "AAHead_yaw" in name_set or "Head_pitch" in name_set:
        return "t1"
    if "Head_Yaw" in name_set or "Head_Pitch" in name_set:
        return "k1"
    return None


def _rotate_vec(
    quat_xyzw: tuple[float, float, float, float],
    vec: tuple[float, float, float],
) -> tuple[float, float, float]:
    qx, qy, qz, qw = quat_xyzw
    vx, vy, vz = vec
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + qy * tz - qz * ty,
        vy + qw * ty + qz * tx - qx * tz,
        vz + qw * tz + qx * ty - qy * tx,
    )


def _quat_mul(
    q1: tuple[float, float, float, float],
    q2: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return normalize_quaternion_xyzw(
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def _invert_pose(
    pos: tuple[float, float, float], quat_xyzw: tuple[float, float, float, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    qx, qy, qz, qw = normalize_quaternion_xyzw(*quat_xyzw)
    inv_q = (-qx, -qy, -qz, qw)
    inv_p = _rotate_vec(inv_q, (-pos[0], -pos[1], -pos[2]))
    return inv_p, inv_q


def _compose_pose(
    pose_ab: tuple[tuple[float, float, float], tuple[float, float, float, float]],
    pose_bc: tuple[tuple[float, float, float], tuple[float, float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    p_ab, q_ab = pose_ab
    p_bc, q_bc = pose_bc
    rot_p = _rotate_vec(q_ab, p_bc)
    return (p_ab[0] + rot_p[0], p_ab[1] + rot_p[1], p_ab[2] + rot_p[2]), _quat_mul(
        q_ab, q_bc
    )


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BoosterProtocolError(f"Booster RPC field {field!r} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise BoosterProtocolError(f"Booster RPC field {field!r} must be finite")
    return result


def _json_object(value: str, field: str) -> dict[str, Any]:
    if not value:
        return {}
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise BoosterProtocolError(f"invalid Booster RPC {field} JSON: {exc}") from exc
    if not isinstance(parsed, dict):
        raise BoosterProtocolError(f"Booster RPC {field} must contain a JSON object")
    return parsed


def _clamp_range(value: float, bounds: tuple[float, float]) -> float:
    return max(float(bounds[0]), min(float(bounds[1]), value))


class BoosterTranslatorCore:
    """Stateful protocol translator for one robot connecting PumaSim <-> Pumanoids demo."""

    def __init__(
        self,
        robot_name: str,
        *,
        robot_type: str = "auto",
        velocity_limits: Mapping[str, tuple[float, float]] | None = None,
    ) -> None:
        self.robot_name = robot_name
        normalized_type = robot_type.lower().strip()
        if normalized_type not in {"auto", *ROBOT_PROFILES}:
            raise ValueError(
                f"robot_type must be 'auto' or one of {tuple(ROBOT_PROFILES)}, got {robot_type!r}"
            )
        self._robot_type: str | None = (
            None if normalized_type == "auto" else normalized_type
        )
        self._velocity_limits = dict(DEFAULT_VELOCITY_LIMITS)
        if velocity_limits:
            self._velocity_limits.update(velocity_limits)

        self.mode: int = WALKING_MODE
        self._head_target: tuple[float, float] = (0.0, 0.0)
        self._odom_origin: tuple[float, float, float] | None = None
        self._last_nav_odom_pose: tuple[float, float, float] | None = None
        self._frame_poses: dict[
            int, tuple[tuple[float, float, float], tuple[float, float, float, float]]
        ] = {}
        self._available_skills: tuple[str, ...] = ()
        self._active_skill: str = ""
        self._skill_state: str = "idle"
        self._latest_kick_reference: BoosterKickReference | None = None

    @property
    def robot_type(self) -> str | None:
        return self._robot_type

    @property
    def available_skills(self) -> tuple[str, ...]:
        return self._available_skills

    @property
    def is_recovery_available(self) -> int:
        return 1 if "get_up" in self._available_skills else 0

    def ensure_robot_type(self, joint_names: Sequence[str]) -> str | None:
        if self._robot_type is None:
            detected = detect_robot_type(joint_names)
            if detected is not None:
                self._robot_type = detected
        return self._robot_type

    def reset_state(self) -> None:
        self.mode = WALKING_MODE
        self._head_target = (0.0, 0.0)
        self._odom_origin = None
        self._last_nav_odom_pose = None
        self._active_skill = ""
        self._skill_state = "idle"
        self._latest_kick_reference = None

    def update_skill_status(self, payload: Mapping[str, Any]) -> None:
        raw_avail = payload.get("available_skills")
        if isinstance(raw_avail, Sequence) and not isinstance(raw_avail, (str, bytes)):
            self._available_skills = tuple(str(item) for item in raw_avail)
        if isinstance(payload.get("active_skill"), str):
            self._active_skill = payload["active_skill"]
        if isinstance(payload.get("state"), str):
            self._skill_state = payload["state"]

    def anchor_odometry_origin(self) -> None:
        if self._last_nav_odom_pose is not None:
            self._odom_origin = self._last_nav_odom_pose
        else:
            self._odom_origin = (0.0, 0.0, 0.0)

    def update_relative_odometry(
        self, x: float, y: float, qx: float, qy: float, qz: float, qw: float
    ) -> BoosterOdometerSample:
        yaw = quaternion_xyzw_to_rpy(qx, qy, qz, qw)[2]
        self._last_nav_odom_pose = (float(x), float(y), yaw)
        if self._odom_origin is None:
            self._odom_origin = self._last_nav_odom_pose
        return relative_odom_from_nav(x, y, qx, qy, qz, qw, self._odom_origin)

    def update_frame_pose(
        self,
        frame_id: int,
        pos: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
    ) -> None:
        self._frame_poses[int(frame_id)] = (
            (float(pos[0]), float(pos[1]), float(pos[2])),
            normalize_quaternion_xyzw(*quat_xyzw),
        )

    def update_head_pose(
        self,
        pos: tuple[float, float, float],
        quat_xyzw: tuple[float, float, float, float],
    ) -> None:
        self.update_frame_pose(FRAME_HEAD, pos, quat_xyzw)

    def update_kick_reference(
        self,
        *,
        ball_x: float,
        ball_y: float,
        direction: float,
        goal_x: float,
        goal_y: float,
        robot_theta_to_field: float,
        power: float,
    ) -> BoosterKickReference:
        ref = BoosterKickReference(
            ball_x=_finite_number(ball_x, "kick.x"),
            ball_y=_finite_number(ball_y, "kick.y"),
            direction=_finite_number(direction, "kick.dir"),
            goal_x=_finite_number(goal_x, "kick.goal_x"),
            goal_y=_finite_number(goal_y, "kick.goal_y"),
            robot_theta_to_field=_finite_number(
                robot_theta_to_field, "kick.robot_theta_to_field"
            ),
            power=_finite_number(power, "kick.power"),
        )
        self._latest_kick_reference = ref
        return ref

    def build_low_state(
        self,
        *,
        imu_quat_xyzw: tuple[float, float, float, float],
        imu_gyro: tuple[float, float, float],
        imu_acc: tuple[float, float, float],
        joint_names: Sequence[str],
        joint_positions: Sequence[float],
        joint_velocities: Sequence[float],
        joint_efforts: Sequence[float],
    ) -> BoosterLowStateSample | None:
        rtype = self.ensure_robot_type(joint_names)
        if rtype is None:
            return None
        profile = ROBOT_PROFILES[rtype]
        pos_map = dict(zip(joint_names, joint_positions, strict=True))
        vel_map = dict(zip(joint_names, joint_velocities, strict=True))
        eff_map = dict(zip(joint_names, joint_efforts, strict=True))
        mode_byte = max(0, self.mode)

        def _samples(order: tuple[str, ...]) -> tuple[BoosterMotorSample, ...]:
            items = []
            for jname in order:
                items.append(
                    BoosterMotorSample(
                        mode=mode_byte,
                        q=float(pos_map.get(jname, 0.0)),
                        dq=float(vel_map.get(jname, 0.0)),
                        ddq=0.0,
                        tau_est=float(eff_map.get(jname, 0.0)),
                    )
                )
            return tuple(items)

        return BoosterLowStateSample(
            rpy=quaternion_xyzw_to_rpy(*imu_quat_xyzw),
            gyro=tuple(float(v) for v in imu_gyro),
            acc=tuple(float(v) for v in imu_acc),
            motor_state_parallel=_samples(profile["parallel_order"]),
            motor_state_serial=_samples(profile["serial_order"]),
        )

    def handle_rpc(self, header: str, body: str) -> BoosterRpcResult:
        try:
            header_data = _json_object(header, "header")
            api_id = header_data.get("api_id")
            if isinstance(api_id, bool) or not isinstance(api_id, int):
                raise BoosterProtocolError("Booster RPC header requires integer api_id")
        except BoosterProtocolError:
            return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_BAD_REQUEST))
        return self.handle_api_call(api_id, body)

    def handle_api_call(self, api_id: int, body: str) -> BoosterRpcResult:
        try:
            body_data = _json_object(body, "body")
            if api_id == CHANGE_MODE_API:
                return self._change_mode(body_data)
            if api_id == MOVE_API:
                return self._move(body_data)
            if api_id == ROTATE_HEAD_API:
                return self._rotate_head(body_data)
            if api_id == ROTATE_HEAD_WITH_DIRECTION_API:
                return self._rotate_head_with_direction(body_data)
            if api_id == GET_FRAME_TRANSFORM_API:
                return self._get_frame_transform(body_data)
            if api_id == GET_MODE_API:
                return BoosterRpcResult(
                    BoosterRpcResponse(
                        RPC_STATUS_SUCCESS,
                        json.dumps({"mode": self.mode}, separators=(",", ":")),
                    )
                )
            if api_id == GET_STATUS_API:
                return BoosterRpcResult(
                    BoosterRpcResponse(
                        RPC_STATUS_SUCCESS,
                        json.dumps(
                            {
                                "current_mode": self.mode,
                                "current_body_control": _BODY_CONTROL_BY_MODE.get(
                                    self.mode, 0
                                ),
                                "current_actions": (
                                    [self._active_skill] if self._active_skill else []
                                ),
                            },
                            separators=(",", ":"),
                        ),
                    )
                )
            if api_id == GET_ROBOT_INFO_API:
                model = (
                    ROBOT_PROFILES[self._robot_type]["model"]
                    if self._robot_type is not None
                    else "UNKNOWN"
                )
                return BoosterRpcResult(
                    BoosterRpcResponse(
                        RPC_STATUS_SUCCESS,
                        json.dumps(
                            {
                                "name": self.robot_name,
                                "nickname": self.robot_name,
                                "version": "pumasim-1.0",
                                "model": model,
                                "serial_number": f"pumasim-{self.robot_name}",
                            },
                            separators=(",", ":"),
                        ),
                    )
                )
            if api_id == RESET_ODOMETRY_API:
                self.anchor_odometry_origin()
                return BoosterRpcResult(
                    BoosterRpcResponse(RPC_STATUS_SUCCESS), reset_odometry=True
                )
            skill_name = SKILL_NAME_BY_API.get(api_id)
            if skill_name is not None:
                return self._dispatch_skill(skill_name, body_data)
        except BoosterProtocolError:
            return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_BAD_REQUEST))

        # Explicit refusal (501) for unimplemented policies/actions; never fabricate success.
        return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_SERVER_REFUSED))

    def _dispatch_skill(
        self, skill_name: str, body_data: dict[str, Any]
    ) -> BoosterRpcResult:
        # TODO(pumasim-policies): Cuando PumaSim publique ``skill_name`` dentro
        # de ``available_skills`` en ``/<robot>/skill_status``, esta rama lo
        # reenviará automáticamente a ``/<robot>/skill_command`` con status 0.
        if skill_name not in self._available_skills:
            return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_SERVER_REFUSED))
        return BoosterRpcResult(
            BoosterRpcResponse(RPC_STATUS_SUCCESS),
            skill_command=(skill_name, dict(body_data)),
        )

    def _change_mode(self, body: dict[str, Any]) -> BoosterRpcResult:
        mode_value = body.get("mode")
        if (
            isinstance(mode_value, bool)
            or not isinstance(mode_value, int)
            or mode_value not in _VALID_MODES
        ):
            raise BoosterProtocolError("ChangeMode requires valid mode integer")
        if mode_value == CUSTOM_MODE:
            return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_SERVER_REFUSED))
        self.mode = mode_value
        cmd_vel = (0.0, 0.0, 0.0) if mode_value in {DAMPING_MODE, PREPARE_MODE} else None
        return BoosterRpcResult(
            BoosterRpcResponse(RPC_STATUS_SUCCESS), cmd_vel=cmd_vel
        )

    def _move(self, body: dict[str, Any]) -> BoosterRpcResult:
        vx = _finite_number(body.get("vx"), "vx")
        vy = _finite_number(body.get("vy"), "vy")
        vyaw = _finite_number(body.get("vyaw"), "vyaw")
        if self.mode not in _LOCOMOTION_MODES:
            return BoosterRpcResult(
                BoosterRpcResponse(RPC_STATUS_CONFLICT), cmd_vel=(0.0, 0.0, 0.0)
            )
        clamped = (
            _clamp_range(vx, self._velocity_limits["vx"]),
            _clamp_range(vy, self._velocity_limits["vy"]),
            _clamp_range(vyaw, self._velocity_limits["wz"]),
        )
        return BoosterRpcResult(
            BoosterRpcResponse(RPC_STATUS_SUCCESS), cmd_vel=clamped
        )

    def _apply_head(self, yaw: float, pitch: float) -> BoosterRpcResult:
        if self._robot_type is None:
            return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_STATE_TRANSITION_FAILED))
        profile = ROBOT_PROFILES[self._robot_type]
        yaw_limits, pitch_limits = profile["head_limits"]
        clamped_yaw = _clamp_range(yaw, yaw_limits)
        clamped_pitch = _clamp_range(pitch, pitch_limits)
        self._head_target = (clamped_yaw, clamped_pitch)
        return BoosterRpcResult(
            BoosterRpcResponse(RPC_STATUS_SUCCESS),
            joint_targets=(profile["head_joints"], (clamped_yaw, clamped_pitch)),
        )

    def _rotate_head(self, body: dict[str, Any]) -> BoosterRpcResult:
        pitch = _finite_number(body.get("pitch"), "pitch")
        yaw = _finite_number(body.get("yaw"), "yaw")
        return self._apply_head(yaw, pitch)

    def _rotate_head_with_direction(self, body: dict[str, Any]) -> BoosterRpcResult:
        pitch_dir = body.get("pitch_direction")
        yaw_dir = body.get("yaw_direction")
        if (
            isinstance(pitch_dir, bool)
            or isinstance(yaw_dir, bool)
            or not isinstance(pitch_dir, int)
            or not isinstance(yaw_dir, int)
            or pitch_dir not in (-1, 0, 1)
            or yaw_dir not in (-1, 0, 1)
        ):
            raise BoosterProtocolError(
                "RotateHeadWithDirection requires integer directions in {-1, 0, 1}"
            )
        cur_yaw, cur_pitch = self._head_target
        return self._apply_head(
            cur_yaw + yaw_dir * _ROTATE_HEAD_DIRECTION_STEP_RAD,
            cur_pitch + pitch_dir * _ROTATE_HEAD_DIRECTION_STEP_RAD,
        )

    def _get_frame_transform(self, body: dict[str, Any]) -> BoosterRpcResult:
        src = body.get("src")
        dst = body.get("dst")
        if (
            isinstance(src, bool)
            or isinstance(dst, bool)
            or not isinstance(src, int)
            or not isinstance(dst, int)
        ):
            raise BoosterProtocolError("GetFrameTransform requires integer src and dst")
        if not (FRAME_BODY <= src <= FRAME_RIGHT_FOOT and FRAME_BODY <= dst <= FRAME_RIGHT_FOOT):
            return BoosterRpcResult(BoosterRpcResponse(RPC_STATUS_BAD_REQUEST))

        identity = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        if src == dst:
            pos, quat = identity
        else:
            src_pose = identity if src == FRAME_BODY else self._frame_poses.get(src)
            dst_pose = identity if dst == FRAME_BODY else self._frame_poses.get(dst)
            if src_pose is None or dst_pose is None:
                return BoosterRpcResult(
                    BoosterRpcResponse(RPC_STATUS_STATE_TRANSITION_FAILED)
                )
            pos, quat = _compose_pose(_invert_pose(*src_pose), dst_pose)

        payload = {
            "position": {"x": pos[0], "y": pos[1], "z": pos[2]},
            "orientation": {
                "x": quat[0],
                "y": quat[1],
                "z": quat[2],
                "w": quat[3],
            },
        }
        return BoosterRpcResult(
            BoosterRpcResponse(
                RPC_STATUS_SUCCESS, json.dumps(payload, separators=(",", ":"))
            )
        )
