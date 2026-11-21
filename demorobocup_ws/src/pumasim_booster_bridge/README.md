# `pumasim_booster_bridge`

ROS 2 translation bridge in `Pumanoids/demorobocup_ws` connecting PumaSim's generic per-robot ROS 2 interfaces to the Booster RoboCup demo (`vision`, `brain`, `game_controller`).

## Responsibility Split

- **PumaSim (`full_ros2topic`)** exposes only generic simulator interfaces:
  - `/clock` (`rosgraph_msgs/msg/Clock`)
  - `/<robot>/ground_truth/odom` (`nav_msgs/msg/Odometry`)
  - `/<robot>/odom` (`nav_msgs/msg/Odometry`)
  - `/<robot>/joint_states` (`sensor_msgs/msg/JointState`, including K1 passive ankle pitch/roll)
  - `/<robot>/imu/data` (`sensor_msgs/msg/Imu`)
  - `/<robot>/head_pose` (`geometry_msgs/msg/PoseStamped`, `<robot>/base_link` -> `<robot>/<head_body>`)
  - `/<robot>/camera/image_raw`, `/<robot>/camera/camera_info`, optional `/<robot>/camera/depth/image_raw`, `/tf`
  - Subscriptions: `/<robot>/cmd_vel` (`geometry_msgs/msg/Twist`), `/<robot>/joint_targets` (`sensor_msgs/msg/JointState`)
- **`pumasim_booster_bridge`** performs all Booster-specific protocol translation:
  - `/<robot>/ground_truth/localization` (`booster_interface/msg/Odometer`): derived from `/<robot>/ground_truth/odom` (`x, y, theta` wrapped in `(-pi, pi]`), kept **separate** from `odometer_state`.
  - `odometer_state` (`booster_interface/msg/Odometer`): zero-origin / resettable (`ResetOdometry 2031`) relative pose derived from `/<robot>/odom`.
  - `low_state` (`booster_interface/msg/LowState`): IMU RPY/gyro/acc from `/<robot>/imu/data` + T1 (23 serial) / K1 (22 parallel crank + 22 serial w/ passive ankle pitch/roll) motor arrays from `/<robot>/joint_states`.
  - `head_pose` (`geometry_msgs/msg/Pose`): unstamped `T_base_head` converted from `/<robot>/head_pose` (`PoseStamped`).
  - `fall_down_recovery_state` (`booster_interface/msg/RawBytesMsg`): `[state, is_recovery_available=0, current_planner_index]` inferred from IMU orientation.
  - `LocoApiTopicReq` / `LocoApiTopicResp` (`booster_msgs/msg/RpcReqMsg` / `RpcRespMsg`, correlated by `uuid`) and topic-service equivalent `booster_rpc_req` / `booster_rpc_resp` (`BoosterApiReqMsg` / `BoosterApiRespMsg`):
    - `2000 ChangeMode`, `2001 Move` -> `/<robot>/cmd_vel` (`409` conflict in Damping/Prepare)
    - `2004 RotateHead`, `2006 RotateHeadWithDirection` -> `/<robot>/joint_targets`
    - `2011 GetFrameTransform` (`base <-> head`), `2017 GetMode`, `2018 GetStatus`, `2022 GetRobotInfo`, `2031 ResetOdometry`
    - Explicit `501` (`kRpcStatusCodeServerRefused`) for unimplemented policies (`2005`, `2007`, `2008`, `2024`, `2025`, `2038`, `100011`, `100012`, `kCustom=3`, unknown IDs).

## `brain` Parameter Loading, Shared GameController, and TF Isolation

`launch/robocup_demo_robot.launch.py` fixes three multi-robot / launch issues:

1. **Node-name YAML key mismatch (`brain_node:` vs `brain_<robot>`)**: `pumasim_booster_bridge/launch_utils.py` (`load_node_parameters_from_yaml`) reads `brain/config/config.yaml` and `config/brain_{t1,k1}.yaml` explicitly into flat ROS parameter dictionaries so `launch_ros` binds them directly to `brain_<robot>` in strict order:
   `base YAML -> T1/K1 overlay -> per-robot launch arguments` (`game.team_id`, `game.player_id`, `game.player_role`, `vision.image_topic`, `vision.depth_image_topic`, `enable_com`, etc.).
2. **Shared GameController**: `/robocup/game_controller` (`game_controller_interface/msg/GameControlData`) is **not** remapped per robot, so a single GameController feeds every `brain_<robot>` instance simultaneously while each keeps its own `team_id`, `player_id`, and `player_role`.
3. **Private per-robot TF for `brain`**: Because `brain` broadcasts fixed frames `odom -> base_link`, multi-robot launches (`namespaced:=true`) remap `/tf`, `tf`, `/tf_static`, `tf_static` for that robot's demo nodes onto `/<robot>/booster/tf` and `/<robot>/booster/tf_static`. Global `/tf` stays reserved for PumaSim's namespaced `<robot>/base_link -> <robot>/camera_optical_frame`.

## Camera Remappings vs. RGB QoS Relay

- **`CameraInfo`**: Pumanoids `vision_node` uses `rclcpp::SensorDataQoS()` (`BEST_EFFORT`), matching PumaSim's `/<robot>/camera/camera_info` directly (`relay_camera_info:=false` default). Set `relay_camera_info:=true` only if launching an older `RELIABLE` `CameraInfo` consumer.
- **RGB (`image_raw`)**: `vision_node` (`image_transport::subscribe`) and `brain_node` default to `RELIABLE`, whereas PumaSim publishes `BEST_EFFORT` (`relay_rgb:=true` preserves exact `(sec, nanosec)` capture stamps). Pass `relay_rgb:=false` when consumers accept `BEST_EFFORT`.

## Launch Commands

Use the PumaSim `full_ros2topic` branch for the generic interfaces and the
Pumanoids `codex/pumasim-booster-bridge` branch for this package. Run the
processes in separate terminals with the same ROS distribution and
`ROS_DOMAIN_ID`. Start with the bridge alone to check the connection.

```bash
# Once, in Pumanoids/demorobocup_ws. Use Humble instead if installed there.
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/pumasim_booster_bridge \
  src/booster_msgs src/booster_ros2_interface --ignore-src -y
colcon build --symlink-install \
  --packages-select booster_msgs booster_interface pumasim_booster_bridge
source install/local_setup.bash
```

For PumaSim, source that same ROS distro and run
`uv sync --locked --python "$(command -v python3)"` from the PumaSim checkout.
In each terminal, source the appropriate setup files and use, for example,
`export ROS_DOMAIN_ID=121`. Only the bridge and clients of Booster messages
need the Pumanoids overlay.

### Single robot (`blue_1`, T1)

```bash
# Terminal 1 (PumaSim)
uv run --locked --no-sync pumanoids-sim run --config single_t1 --headless --ros2

# Terminal 2 (Pumanoids bridge only)
ros2 launch pumasim_booster_bridge booster_bridge.launch.py \
  robots:=blue_1 robot_type:=t1 layout:=root relay_rgb:=true
```

Replace `single_t1` with `single_k1` and `robot_type:=t1` with
`robot_type:=k1` to run K1. In a third terminal with the Pumanoids overlay,
verify the state and the absolute/relative localization outputs:

```bash
ros2 topic list -t
ros2 topic echo /low_state --once
ros2 topic echo /blue_1/ground_truth/localization --once
ros2 topic echo /odometer_state --once
ros2 topic hz /boostercamera/head/rgb
```

To verify translation in the other direction, start
`ros2 topic echo /LocoApiTopicResp` in an observer terminal and send a head
request. The bridge forwards it to `/blue_1/joint_targets`. Observe the
measured response on `/blue_1/head_pose` or on the translated `/head_pose`.

```bash
ros2 topic pub --once /LocoApiTopicReq booster_msgs/msg/RpcReqMsg \
  '{uuid: head-smoke-1, header: "{\"api_id\":2004,\"expect_response\":true}", body: "{\"pitch\":0.15,\"yaw\":0.4}"}'
```

When the external `brain` and `vision` packages and models are installed,
stop the bridge-only launch and use the combined launch. It starts its own
bridge. The GameController receiver runs separately and publishes one shared
`/robocup/game_controller` stream.

```bash
ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=blue_1 robot_type:=t1 namespaced:=false \
  team_id:=70 player_id:=1 player_role:=striker \
  enable_vision:=true enable_brain:=true
```

The launch also accepts `detection_model_path` and `segmentation_model_path`
when the vision models are installed outside their package defaults.
With the GameController package installed and a referee broadcasting match
state, start one receiver in a separate terminal on the same ROS domain:

```bash
ros2 run game_controller game_controller_node --ros-args -p port:=3838
```

Set the team's `team_id` and each `player_id` to match the referee configuration.

### Multi-robot demo

Start a PumaSim scenario YAML with a walking T1 named `blue_1` and a walking
K1 named `red_1`, then pass the YAML path to `pumanoids-sim run --config`.
Run each launch below in its own terminal. Enable vision on both if available.

```bash
ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=blue_1 robot_type:=t1 namespaced:=true \
  team_id:=70 player_id:=1 player_role:=striker \
  enable_vision:=false enable_brain:=true

ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=red_1 robot_type:=k1 namespaced:=true \
  team_id:=71 player_id:=3 player_role:=goal_keeper \
  enable_vision:=false enable_brain:=true
```

### 10-robot bridge only (5v5 benchmark / multi-robot fleet)

```bash
ros2 launch pumasim_booster_bridge booster_bridge.launch.py \
  robots:=blue_1,blue_2,blue_3,blue_4,blue_5,red_1,red_2,red_3,red_4,red_5 \
  layout:=namespaced single_process:=true relay_rgb:=true
```

## ROS 2 Jazzy Build Status & External Dependencies

- Compiles cleanly on Ubuntu 24.04 / ROS 2 Jazzy: `booster_msgs`, `booster_interface`, `game_controller_interface`, `vision_interface`, `game_controller`, `pumasim_booster_bridge`.
- Blocked in `demorobocup_ws` without external vendor SDK / GPU toolchain:
  - `vision`: `src/model/trt/CMakeLists.txt:4` invokes `enable_language(CUDA)` requiring `nvcc`, and `vision/src/CMakeLists.txt:9` links `booster_robotics_sdk.a`.
  - `brain`: `CMakeLists.txt:15` requires `rerun_sdk`, and `brain/src/robot_client.cpp:3` includes `<booster_internal/robot/b1/b1_loco_internal_api.hpp>`.
- Runtime tests were executed on ROS 2 Jazzy (Humble container/installation not present on this host).

## Test the bridge from the PumaSim environment

With ROS and the Pumanoids bridge overlay sourced, run from PumaSim. Use the
path to your checkout of Pumanoids. These tests use ROS processes separately
from the simulator. Add `game_controller_interface` to the overlay build for
the GameController test.

```bash
export PUMASIM_ROOT="$PWD"
PUMASIM_TEST_ROS2=1 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \
  uv run --locked --no-sync pytest -c pyproject.toml -q \
  "$HOME/Pumanoids/demorobocup_ws/src/pumasim_booster_bridge/test/test_booster_protocol.py" \
  "$HOME/Pumanoids/demorobocup_ws/src/pumasim_booster_bridge/test/test_booster_ros2_e2e.py"
```

The separate launch regression tests in `test_demo_launch_and_isolation.py`
also require the ROS launch Python dependencies. Set
`PUMASIM_UPSTREAM_DEMO_INSTALL` to opt into the test using an independently
built official Booster `brain`. That test uses the official executable with
vision disabled. Validation of the exact Pumanoids demo with vision is pending.

## Historical 5v5 measurements (Jazzy, Windows GPU backend)

The tables below are the original reported figures. The raw artifacts are
unavailable for review and the reported Tx column contains values inconsistent
with the configured simulated FPS and RTF. Treat these as preliminary evidence
of possible overhead. Use fresh JSON reports for acceptance and label the
receiver stage when quoting rates or latency.

Run sequential comparisons with the
[PumaSim benchmark guide](https://github.com/LIRA-UNAM/PumaSim/blob/full_ros2topic/docs/multicamera-benchmark.md).
For a same-host run, start the external bridge through PumaSim's
`uv run --locked --no-sync python tools/ros2_local.py ros2 launch ...` in
the same ROS domain to match the benchmark's transport. Compare without the
bridge, with the bridge and `relay_rgb:=false`, and with `relay_rgb:=true`.
The standard receiver measures PumaSim's original RGB topics. Measuring relay
delivery needs subscriptions to the Booster outputs and separate results.

Commands run with `tools/benchmark_multicamera.py --require-ros --robots-per-team 5 --warmup 8 --seconds 180 --matched-sim-time` (10 robots, 10× 640×480 RGB cameras, `single_process:=true`, `relay_rgb:=true` with real compatibility subscribers receiving all 10 robots' outputs):

### Campaign A — Habitual benchmark command `[0.3, 0.0, 0.3]` (T1 near policy deadzone: `0.029–0.034 m` disp; K1 `0.127–0.139 m`)

| Case | Scenario | Wall (s) | RTF | Tx min (Hz) | Rx min (Hz) | Snapshot->Raw Rx p50/p95 (ms) | Raw->Relay Hop p50/p95 (ms) | State SHA256 (prefix) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Baseline `dd70adb` | `t1_vs_t1` | 244.58 | 0.736 | 17.32 | 12.74 | 146.94 / 193.52 | — | `e2eb0b864456` |
| Generic `full_ros2topic` | `t1_vs_t1` | 262.04 | 0.687 | 16.46 | 11.30 | 166.86 / 224.88 | — | `e2eb0b864456` |
| Generic + 10r Bridge + Relay + Rx | `t1_vs_t1` | 382.10 | 0.471 | 17.41 | 8.19 | 220.24 / 291.07 | 19.57 / 32.33 | `e2eb0b864456` |
| Baseline `dd70adb` | `k1_vs_k1` | 184.11 | 0.978 | 12.45 | 12.16 | 134.38 / 175.51 | — | `d470daa02d58` |
| Generic `full_ros2topic` | `k1_vs_k1` | 225.06 | 0.800 | 13.97 | 11.16 | 161.18 / 215.82 | — | `d470daa02d58` |
| Generic + 10r Bridge + Relay + Rx | `k1_vs_k1` | 338.38 | 0.532 | 17.12 | 9.10 | 204.90 / 270.70 | 15.67 / 27.04 | `d470daa02d58` |

*Bridge process footprint (`campA`, both scenarios total): `801.98 s` CPU / `145.9 MB` RSS; compatibility receiver `766.16 s` CPU / `120.7 MB` RSS.*

### Campaign B — Effective walking command `[0.6, 0.0, 0.3]` (T1 `1.56–4.72 m` disp, mean `3.30 m`; K1 `1.01–4.82 m` disp, mean `2.70 m`)

| Case | Scenario | Wall (s) | RTF | Tx min (Hz) | Rx min (Hz) | Snapshot->Raw Rx p50/p95 (ms) | Raw->Relay Hop p50/p95 (ms) | State SHA256 (prefix) |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Baseline `dd70adb` | `t1_vs_t1` | 201.05 | 0.895 | 14.91 | 13.34 | 152.77 / 223.80 | — | `cd754e5ca479` |
| Generic `full_ros2topic` | `t1_vs_t1` | 242.15 | 0.743 | 16.13 | 11.98 | 156.39 / 206.78 | — | `cd754e5ca479` |
| Generic + 10r Bridge + Relay + Rx | `t1_vs_t1` | 292.69 | 0.615 | 15.98 | 9.82 | 196.48 / 256.13 | 32.81 / 51.16 | `cd754e5ca479` |
| Baseline `dd70adb` | `k1_vs_k1` | 245.30 | 0.734 | 16.32 | 11.97 | 148.09 / 188.50 | — | `e32cd36b55e1` |
| Generic `full_ros2topic` | `k1_vs_k1` | 280.10 | 0.643 | 18.11 | 11.63 | 167.89 / 224.68 | — | `e32cd36b55e1` |
| Generic + 10r Bridge + Relay + Rx | `k1_vs_k1` | 354.82 | 0.507 | 18.79 | 9.53 | 206.44 / 271.85 | 10.05 / 20.11 | `e32cd36b55e1` |

*Bridge process footprint (`campB`, both scenarios total): `730.07 s` CPU / `192.6 MB` RSS; compatibility receiver `663.05 s` CPU / `148.0 MB` RSS.*
