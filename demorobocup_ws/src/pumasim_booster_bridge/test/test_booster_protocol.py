import json
import math

import pytest
from pumasim_booster_bridge.booster_protocol import (
    CHANGE_MODE_API,
    DAMPING_MODE,
    GET_FRAME_TRANSFORM_API,
    GET_MODE_API,
    GET_ROBOT_INFO_API,
    GET_STATUS_API,
    GET_UP_API,
    K1_PARALLEL_JOINT_ORDER,
    PREPARE_MODE,
    RESET_ODOMETRY_API,
    RL_KICK_BALL_API,
    ROTATE_HEAD_API,
    ROTATE_HEAD_WITH_DIRECTION_API,
    RPC_STATUS_BAD_REQUEST,
    RPC_STATUS_CONFLICT,
    RPC_STATUS_SERVER_REFUSED,
    RPC_STATUS_SUCCESS,
    SOCCER_MODE,
    T1_SERIAL_JOINT_ORDER,
    VISUAL_KICK_API,
    WALKING_MODE,
    BoosterTranslatorCore,
    ground_truth_odom_to_localization,
    recovery_state_from_imu,
    wrap_angle,
)


def _yaw_quat_xyzw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_ground_truth_odom_to_localization_and_pi_wrap() -> None:
    sample = ground_truth_odom_to_localization(2.5, -1.25, *_yaw_quat_xyzw(0.75))
    assert (sample.x, sample.y, sample.theta) == pytest.approx((2.5, -1.25, 0.75))

    near_neg_pi = ground_truth_odom_to_localization(0.0, 0.0, *_yaw_quat_xyzw(-math.pi + 1e-4))
    assert near_neg_pi.theta == pytest.approx(-math.pi + 1e-4, abs=1e-5)

    exact_neg_pi = wrap_angle(-math.pi)
    assert exact_neg_pi == pytest.approx(math.pi)


def test_relative_odometry_rotates_into_initial_heading_and_resets() -> None:
    core = BoosterTranslatorCore("blue_1", robot_type="t1")
    origin_q = _yaw_quat_xyzw(math.pi / 2.0)
    first = core.update_relative_odometry(3.0, -2.0, *origin_q)
    assert (first.x, first.y, first.theta) == pytest.approx((0.0, 0.0, 0.0))

    # Move +1 m along +Y (robot's initial forward heading at +pi/2)
    step = core.update_relative_odometry(3.0, -1.0, *origin_q)
    assert (step.x, step.y, step.theta) == pytest.approx((1.0, 0.0, 0.0))

    res = core.handle_api_call(RESET_ODOMETRY_API, "{}")
    assert res.response.status == RPC_STATUS_SUCCESS
    assert res.reset_odometry is True
    after_reset = core.update_relative_odometry(3.0, -1.0, *origin_q)
    assert (after_reset.x, after_reset.y, after_reset.theta) == pytest.approx((0.0, 0.0, 0.0))


def test_low_state_layouts_for_t1_and_k1_passive_ankles() -> None:
    t1 = BoosterTranslatorCore("blue_1", robot_type="auto")
    t1_low = t1.build_low_state(
        imu_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        imu_gyro=(0.1, 0.2, 0.3),
        imu_acc=(0.0, 0.0, 9.81),
        joint_names=T1_SERIAL_JOINT_ORDER,
        joint_positions=[float(i) * 0.01 for i in range(len(T1_SERIAL_JOINT_ORDER))],
        joint_velocities=[0.0] * len(T1_SERIAL_JOINT_ORDER),
        joint_efforts=[1.0] * len(T1_SERIAL_JOINT_ORDER),
    )
    assert t1.robot_type == "t1"
    assert t1_low is not None
    assert len(t1_low.motor_state_parallel) == 0
    assert len(t1_low.motor_state_serial) == 23

    k1_all_names = list(K1_PARALLEL_JOINT_ORDER) + [
        "Left_Ankle_Pitch",
        "Left_Ankle_Roll",
        "Right_Ankle_Pitch",
        "Right_Ankle_Roll",
    ]
    k1_positions = [0.1] * 22 + [-0.4, 0.05, -0.4, -0.05]
    k1_efforts = [2.0] * 22 + [0.0, 0.0, 0.0, 0.0]
    k1 = BoosterTranslatorCore("red_1", robot_type="auto")
    k1_low = k1.build_low_state(
        imu_quat_xyzw=(0.0, 0.0, 0.0, 1.0),
        imu_gyro=(0.0, 0.0, 0.0),
        imu_acc=(0.0, 0.0, 9.81),
        joint_names=k1_all_names,
        joint_positions=k1_positions,
        joint_velocities=[0.0] * len(k1_all_names),
        joint_efforts=k1_efforts,
    )
    assert k1.robot_type == "k1"
    assert k1_low is not None
    assert len(k1_low.motor_state_parallel) == 22
    assert len(k1_low.motor_state_serial) == 22
    assert k1_low.motor_state_serial[14].q == pytest.approx(-0.4)
    assert k1_low.motor_state_serial[15].q == pytest.approx(0.05)
    assert k1_low.motor_state_serial[14].tau_est == 0.0


def test_recovery_state_and_rpc_contract_including_honest_refusals() -> None:
    upright = recovery_state_from_imu(0.0, 0.0, 0.0, 1.0, SOCCER_MODE)
    assert (upright.state, upright.is_recovery_available, upright.current_planner_index) == (0, 0, 20)
    # 90 deg roll -> fallen
    fallen = recovery_state_from_imu(math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4), WALKING_MODE)
    assert (fallen.state, fallen.is_recovery_available, fallen.current_planner_index) == (2, 0, 1)

    core = BoosterTranslatorCore("blue_1", robot_type="t1")
    # Default walking mode allows Move and clamps to policy bounds
    move_res = core.handle_rpc('{"api_id":2001}', '{"vx":5.0,"vy":-0.2,"vyaw":0.3}')
    assert move_res.response.status == RPC_STATUS_SUCCESS
    assert move_res.cmd_vel == (1.0, -0.2, 0.3)

    # ChangeMode(PREPARE) zeros velocity and blocks Move with 409 Conflict
    prep_res = core.handle_rpc('{"api_id":2000}', json.dumps({"mode": PREPARE_MODE}))
    assert prep_res.response.status == RPC_STATUS_SUCCESS
    assert prep_res.cmd_vel == (0.0, 0.0, 0.0)
    conflict = core.handle_rpc('{"api_id":2001}', '{"vx":0.3,"vy":0.0,"vyaw":0.0}')
    assert conflict.response.status == RPC_STATUS_CONFLICT
    assert conflict.cmd_vel == (0.0, 0.0, 0.0)

    # RotateHead & RotateHeadWithDirection
    head_res = core.handle_api_call(ROTATE_HEAD_API, '{"pitch":0.4,"yaw":9.0}')
    assert head_res.response.status == RPC_STATUS_SUCCESS
    assert head_res.joint_targets == (("AAHead_yaw", "Head_pitch"), (1.57, 0.4))
    step_res = core.handle_api_call(ROTATE_HEAD_WITH_DIRECTION_API, '{"pitch_direction":1,"yaw_direction":-1}')
    assert step_res.response.status == RPC_STATUS_SUCCESS
    assert step_res.joint_targets[1] == pytest.approx((1.52, 0.45))

    # GetFrameTransform using cached head pose
    core.update_head_pose((0.01, 0.0, 0.42), (0.0, 0.0, 0.0, 1.0))
    tf_res = core.handle_api_call(GET_FRAME_TRANSFORM_API, '{"src":0,"dst":1}')
    assert tf_res.response.status == RPC_STATUS_SUCCESS
    assert json.loads(tf_res.response.body)["position"]["z"] == pytest.approx(0.42)

    # Query APIs
    assert json.loads(core.handle_api_call(GET_MODE_API, "").response.body)["mode"] == PREPARE_MODE
    assert json.loads(core.handle_api_call(GET_STATUS_API, "").response.body)["current_body_control"] == 2
    assert json.loads(core.handle_api_call(GET_ROBOT_INFO_API, "").response.body)["model"] == "T1"

    # Honest 501 refusals for unavailable policies & kCustom
    for unsupported in (GET_UP_API, VISUAL_KICK_API, RL_KICK_BALL_API, 999999):
        refused = core.handle_api_call(unsupported, "{}")
        assert refused.response.status == RPC_STATUS_SERVER_REFUSED
    assert core.handle_api_call(CHANGE_MODE_API, '{"mode":3}').response.status == RPC_STATUS_SERVER_REFUSED

    # Bad JSON / invalid values -> 400
    assert core.handle_rpc("not-json", "{}").response.status == RPC_STATUS_BAD_REQUEST
    assert core.handle_api_call(DAMPING_MODE * 0 + CHANGE_MODE_API, '{"mode":99}').response.status == RPC_STATUS_BAD_REQUEST
