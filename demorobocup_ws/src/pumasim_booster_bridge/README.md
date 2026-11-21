# `pumasim_booster_bridge`

ROS 2 translation bridge in `Pumanoids/demorobocup_ws` connecting PumaSim's generic per-robot ROS 2 interfaces to the Booster RoboCup demo (`vision`, `brain`, `game_controller`).

See the [simulation guide](../../SIMULATION.md) for complete build and run
commands, scenario pairings, localization and Rerun.

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
  - `/<robot>/ground_truth/localization` (`booster_interface/msg/Odometer`): field pose as `x, y, theta`, with heading wrapped in `(-pi, pi]`. Uses `/<robot>/ground_truth/team_odom` once the first team-frame sample arrives and uses `/<robot>/ground_truth/odom` before that. The global topic remains the fixed-world reference. This pose is separate from `odometer_state`.
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

## Natural Workflow with `--sim` (`scripts/build.sh` and `scripts/start.sh`)

You can compile and launch directly using the native `demorobocup_ws` scripts with the simulation flag, leaving physical robot defaults unchanged when omitted:

```bash
cd /mnt/d/Repositorios/Pumanoids/demorobocup_ws
source /opt/ros/jazzy/setup.bash

# Build (auto-detects ONNXRUNTIME_ROOT, BOOSTER_SDK_INCLUDE_DIR, behaviortree_cpp,
# and /opt/rerun_sdk or ~/.local/rerun_sdk if installed)
./scripts/build.sh --sim

# Start simulation bridge + vision ONNX + brain + game_controller for one robot
# (default: internal SelfLocate localization; add use_mujoco_localization:=true or
#  localization:=mujoco to feed /<robot>/ground_truth/localization directly to brain)
./scripts/start.sh --sim robot:=blue_1 robot_type:=t1 role:=striker number_of_players:=1 gc_ip:=127.0.0.1

# Stop demo/bridge processes cleanly without root or killing PumaSim
./scripts/stop.sh --sim
```

Individual component scripts also accept `--sim` (`scripts/start_vision.sh --sim`, `scripts/start_brain.sh --sim ...`, `scripts/start_game_controller.sh --sim`), and native launch files accept `sim:=true` (`vision/launch/launch.py`, `brain/launch/launch.py`, `game_controller/launch/launch.py`). `brain/launch/launch.py` and `robocup_demo_robot.launch.py` expose `use_mujoco_localization:=false|true` (sets `locator.use_mujoco_localization` and `locator.mujoco_localization_topic`, defaulting to `/<robot_name>/ground_truth/localization`). `game_controller` defaults `enable_ip_white_list` to `false` when `sim:=true` (overridable via `gc_whitelist:=true gc_whitelist_ips:=...`).

For multi-robot simulation on a single host, `scripts/start.sh --sim robots:=blue_1:70:1:goal_keeper:left,blue_2:70:2:striker:right,...` launches [robocup_demo_match.launch.py](launch/robocup_demo_match.launch.py) (1 shared `game_controller_node`, 1 `booster_bridge` with `single_process:=true layout:=namespaced`, 1 `match_supervisor` reading `/robocup/game_controller` to update PumaSim `/<robot>/set_field_flip` and sideline `/<robot>/set_pose`, and N isolated `vision`/`brain` instances communicating over UDP `SO_REUSEPORT`).

## Building `vision`, `brain`, and `pumasim_booster_bridge` for PumaSim (Jazzy)

From `demorobocup_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/pumasim_booster_bridge \
  src/booster_msgs src/booster_ros2_interface src/vision src/brain --ignore-src -y

# 1. Message packages + bridge
colcon build --symlink-install \
  --packages-select booster_msgs booster_interface vision_interface game_controller_interface pumasim_booster_bridge
source install/local_setup.bash

# 2. Vision (ONNX detection + segmentation, no CUDA/TensorRT required)
ONNXRUNTIME_ROOT=/mnt/d/Repositorios/BoosterSoccerSim/.upstream/onnxruntime-linux-x64-1.23.2 \
  colcon build --packages-select vision --symlink-install \
  --cmake-args -DENABLE_ONNX=ON

# 3. Brain (simulation profile: omits private booster_internal; keeps rerun_sdk enabled when installed)
colcon build --packages-select brain --symlink-install \
  --cmake-args \
  -DPUMASIM_SIMULATION_PROFILE=ON \
  -DENABLE_RERUN=ON \
  -DCMAKE_PREFIX_PATH="$HOME/.local/rerun_sdk" \
  -DBOOSTER_SDK_INCLUDE_DIR=/mnt/d/Repositorios/BoosterSoccerSim/.upstream/booster_robotics_sdk/include \
  -Dbehaviortree_cpp_DIR=/mnt/d/Repositorios/BoosterSoccerSim/.upstream/robocup_demo/install_jazzy_full/behaviortree_cpp/share/behaviortree_cpp/cmake

source install/setup.bash
```

Notes on build variants:

- Physical robot builds keep defaults (`ENABLE_TENSORRT=ON` in `vision`, `ENABLE_RERUN=ON` and `ENABLE_BOOSTER_INTERNAL_SDK=ON` in `brain`).
- In simulation builds, missing `ONNXRUNTIME_ROOT` / headers or missing `.onnx` files fail explicitly; `RobotClient::walkMode()` returns `-1` with an explicit error log instead of stubbing private internal APIs.
- ONNX simulation models bundled in `src/vision/model/`:
  - Detection: `model/sim_data_det_0126.onnx` (`sha256: 4410f041...`, `320x320` FP32 input, `CPUExecutionProvider`)
  - Segmentation: `model/sim_data_seg_0126.onnx` (`sha256: fdcfa2cb...`, `320x320` FP32 input, `CPUExecutionProvider`)

## Running Reproducible `single_t1` Modes

In every terminal, source ROS 2 Jazzy + `demorobocup_ws/install/setup.bash` (for PumaSim use `uv sync --locked --python "$(command -v python3)"` inside PumaSim), and set the same `ROS_DOMAIN_ID` (and `ros2_local.py` SHM profile when running locally on the same host).

1. **Terminal 1 — PumaSim (`single_t1`, fixed)**

```bash
cd /mnt/d/Repositorios/PumaSim
ROS_DOMAIN_ID=121 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \
  uv run --locked --no-sync python tools/ros2_local.py \
  pumanoids-sim run --config single_t1 --headless --ros2
```

2. **Mode A (Control) — Bridge only**

```bash
cd /mnt/d/Repositorios/Pumanoids/demorobocup_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ROS_DOMAIN_ID=121 ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=blue_1 robot_type:=t1 namespaced:=false \
  enable_vision:=false enable_brain:=false
```

3. **Mode B — Bridge + ONNX Vision**

```bash
ROS_DOMAIN_ID=121 ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=blue_1 robot_type:=t1 namespaced:=false \
  enable_vision:=true enable_brain:=false \
  detection_model_path:=model/sim_data_det_0126.onnx \
  segmentation_model_path:=model/sim_data_seg_0126.onnx
```

4. **Mode C — Full Closed Loop (Bridge + ONNX Vision + Pumanoids `brain` + GameController)**

```bash
ROS_DOMAIN_ID=121 ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=blue_1 robot_type:=t1 namespaced:=false \
  team_id:=70 player_id:=1 player_role:=striker number_of_players:=1 \
  game_control_ip:=127.0.0.1 enable_vision:=true enable_brain:=true
```

Runtime notes:
- Start `/robocup/game_controller` (or `ros2 run game_controller game_controller_node --ros-args -p port:=3838`) so `brain` enters `PLAY`.
- `brain` prints high-volume console status in its startup burst; keep process logs directed to a terminal or file rather than an un-drained small pipe.
- Runtime tests were executed on ROS 2 Jazzy (Humble container/installation not present on this host).

## Running Python Scripts & Tests with `uv`

`Pumanoids` / `pumasim_booster_bridge` does not define a separate `pyproject.toml` or `uv.lock`; ROS 2 package dependencies remain in `package.xml` and C++/ROS binaries build via `colcon` and `rosdep`. Run Python scripts and pytest suites with `uv run --no-project --python /usr/bin/python3` after sourcing ROS Jazzy and the workspace overlay:

```bash
cd /mnt/d/Repositorios/Pumanoids/demorobocup_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Sanity check ROS + Booster Python imports under uv
uv run --no-project --python /usr/bin/python3 python3 -c \
  "import rclpy, booster_interface.msg, booster_msgs.msg; print('OK:', rclpy.__file__)"

# Unit + launch regression tests (note: pytest>=7,<8 matches Jazzy's launch_testing plugin)
uv run --no-project --python /usr/bin/python3 --with 'pytest>=7,<8' --with pyyaml pytest -q \
  src/pumasim_booster_bridge/test/test_booster_protocol.py \
  src/pumasim_booster_bridge/test/test_demo_launch_and_isolation.py

# Full ROS 2 + MuJoCo E2E suite (including closed-loop Pumanoids vision + brain)
PUMASIM_ROOT=/mnt/d/Repositorios/PumaSim \
PUMASIM_TEST_ROS2=1 MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \
  uv run --no-project --python /usr/bin/python3 --with 'pytest>=7,<8' --with pyyaml pytest -q \
  src/pumasim_booster_bridge/test/test_booster_protocol.py \
  src/pumasim_booster_bridge/test/test_booster_ros2_e2e.py \
  src/pumasim_booster_bridge/test/test_demo_launch_and_isolation.py
```

## Measured `single_t1` Load Comparison (Jazzy, 15s steady-state window)

Measured sequentially against identical `PumaSim single_t1 --headless --ros2` (`640x480` calibrated RGB camera, `CPUExecutionProvider`):

| Configuration | Wall (s) | Sim (s) | RTF | Raw Img Rx (Hz) | Booster RGB Rx (Hz) | Det Pub (Hz) / Inf (ms) | Seg Pub (Hz) / Inf (ms) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Bridge only (`single_t1`) | 15.0 | 15.004 | 1.000 | 29.80 | 29.87 | — | — |
| Full ONNX Vision + Brain (`single_t1`) | 15.0 | 15.010 | 1.001 | 18.93 | 19.00 | 11.13 Hz / 53.97 ms | 11.93 Hz / 71.65 ms |

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
