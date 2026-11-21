# Run the RoboCup demo with PumaSim

PumaSim runs physics, walking and cameras. This workspace runs the same
RoboCup demo packages used for the robot, with simulation inputs supplied by
`pumasim_booster_bridge`. The adapter translates regular ROS 2 topics into
the demo's Booster interfaces and routes walking and head commands back to the
right simulated robot. Run the two repositories in separate terminals.

## Requirements and use of uv

| Environment | ROS underlay and system Python | Demo backend |
| --- | --- | --- |
| Ubuntu 24.04 or WSL development | ROS 2 Jazzy, Python 3.12 | ONNX Runtime C++ for simulation |
| Ubuntu 22.04 development | ROS 2 Humble, Python 3.10 | ONNX Runtime C++ for simulation |
| Physical Booster robot | Its deployed ROS 2 Humble environment and matching system Python | Existing Jetson/CUDA/TensorRT and Booster SDK setup |

Use the same ROS distribution and `ROS_DOMAIN_ID` in both simulation
terminals. `rclpy`, generated message bindings and ROS 2 launch come from the
ROS installation and the colcon overlay. Their Python ABI must match the
system interpreter. For **Python tests and helpers** in this workspace, use
`uv run --no-project --python /usr/bin/python3` with any required
`--with` dependencies. The bridge's runtime YAML dependency is
`python3-yaml` through rosdep. The pytest examples add PyYAML and pytest to
the temporary uv test environment.

Use `rosdep` and `colcon` for ROS packages, native dependencies and
generated messages. Vision links **ONNX Runtime C++**, so installing the
Python `onnxruntime` or `opencv-python` wheels does not supply the native
dependencies it builds against. The optional vision model-conversion tools
have their own requirements and are outside the demo runtime. PumaSim has a
separate lockfile for its Python application. Sync it from the PumaSim
checkout with uv and the same system interpreter used by ROS.

## Build the demo

From `demorobocup_ws`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install build-essential cmake libopencv-dev libpcl-dev libeigen3-dev libyaml-cpp-dev libceres-dev
```

Install the native dependencies needed by the selected profile:

- **Simulation vision.** Install the ONNX Runtime C++ archive for your
  architecture with headers and shared libraries. The tested build used
  [ONNX Runtime 1.23.2](https://github.com/microsoft/onnxruntime/releases)
  on Linux x86-64. Set `ONNXRUNTIME_ROOT` to the unpacked directory
  containing `include/` and `lib/`, or install it at `/opt/onnxruntime`.
  The demo loads detection `sim_data_det_0126.onnx` and segmentation
  `sim_data_seg_0126.onnx` from the vision package.
- **Brain.** Install BehaviorTree.CPP as a ROS package or make its CMake
  package visible via the sourced overlay. Install the public Booster SDK
  headers or set `BOOSTER_SDK_INCLUDE_DIR` to the directory containing
  `booster/robot/b1/b1_api_const.hpp`. The simulation profile uses these
  public API declarations and disables the private SDK.
- **Optional Rerun.** Install the C++ SDK before compiling if you want
  `brain` to write recordings or stream to the Viewer. See below.

```bash
# Set these if your dependencies are in custom paths.
export ONNXRUNTIME_ROOT=/opt/onnxruntime
# export BOOSTER_SDK_INCLUDE_DIR=/path/to/booster_robotics_sdk/include
./scripts/build.sh --sim
source install/setup.bash
```

On constrained hosts, pass `--parallel-workers 2` to the build script.
To switch between `--sim` and the robot profile, start in a fresh shell, source
the appropriate ROS underlay and clean the generated `build/`,
`install/` and `log/` directories of **this workspace** before building.
CMake caches the vision backend and SDK selection. On the robot, use the
existing `./scripts/build.sh`, `./scripts/start.sh` and `./scripts/stop.sh`
without the simulation flag.

## Start the simulator and the demo

### Terminal 1 · PumaSim

From the PumaSim checkout:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=121
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/ros2_ws/config/fastdds_local_shm.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
uv sync --locked --python /usr/bin/python3
uv run --locked --no-sync pumanoids-sim run --config t1_vs_t1_1v1 --headless --ros2
```

Replace the scenario using the examples below. Change `--headless` to
`--visualizer viser` for the browser viewer. Omit both for MuJoCo's viewer.
Add `--camera-preview blue_1` or `--camera-preview red_1` if you want to inspect
one robot's RGB locally. On a supported headless host, use `MUJOCO_GL=egl`
and `PYOPENGL_PLATFORM=egl`. The scene publishes its cameras in all three
viewer modes. See the [PumaSim scenario guide](https://github.com/LIRA-UNAM/PumaSim/blob/full_ros2topic/docs/pumanoids-demo.md).

Every bundled scenario spawns the robots outside a touchline in their own
half, facing inward. The default roster matches those spawn positions. For a
single robot the supplied command uses `pos:=left`.

### Terminal 2 · Pumanoids

From `demorobocup_ws`, source the same ROS distribution and workspace
overlay. If using a custom DDS profile, export the same RMW and profile
settings in both terminals before launch. The optional PumaSim local SHM
profile is intended for processes on the same host.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=121
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# Adjust the path if your checkouts are in different directories.
export FASTRTPS_DEFAULT_PROFILES_FILE="$(cd ../.. && pwd)/PumaSim/ros2_ws/config/fastdds_local_shm.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
```

**One T1** with PumaSim `single_t1`:

```bash
./scripts/start.sh --sim robot_name:=blue_1 robot_type:=t1 \
  team_id:=70 player_id:=1 role:=striker pos:=left number_of_players:=1
```

**One K1** with PumaSim `single_k1`:

```bash
./scripts/start.sh --sim robot_name:=blue_1 robot_type:=k1 \
  team_id:=70 player_id:=1 role:=striker pos:=left number_of_players:=1
```

**T1 vs T1 or K1 vs K1 (1v1).** Run `t1_vs_t1_1v1` or `k1_vs_k1_1v1`
in PumaSim. Run the matching command below in Pumanoids.

```bash
./scripts/start.sh --sim 'robots:=blue_1:70:1:striker:left,red_1:71:1:striker:left' \
  robot_type:=t1 use_mujoco_localization:=true

./scripts/start.sh --sim 'robots:=blue_1:70:1:striker:left,red_1:71:1:striker:left' \
  robot_type:=k1 use_mujoco_localization:=true
```

**Same-model 5v5** with PumaSim `t1_vs_t1` or `match_5v5`:

```bash
roster=blue_1,blue_2,blue_3,blue_4,blue_5,red_1,red_2,red_3,red_4,red_5
./scripts/start.sh --sim "robots:=$roster" robot_type:=t1 use_mujoco_localization:=true
```

For `k1_vs_k1`, set `robot_type:=k1` in that command. You can start a subset
of demo players by passing only their names. All robots in the PumaSim
scenario are still simulated. For a roster you will reuse, save the teams,
IDs and model selections as YAML, then run `./scripts/start.sh --sim
roster:=/absolute/path/to/roster.yaml`. Here is the K1 1v1 roster:

```yaml
robots:
  - {name: blue_1, team_id: 70, player_id: 1, role: striker, pos: left, robot_type: k1}
  - {name: red_1, team_id: 71, player_id: 1, role: striker, pos: left, robot_type: k1}
```

The CSV fields are `name:team_id:player_id:role:pos:robot_type`. Bare names
use team 70 for blue and 71 for red. Player 1 defaults to goalkeeper,
remaining players to striker, and `pos` alternates by player ID.

### Choose localization, game state and components

- **Run the real demo's localizer.** Use `use_mujoco_localization:=false`
  (the default). `brain` uses its internal `SelfLocate`, camera detections and
  relative `odometer_state`. This is how to exercise the localization path
  that will run on the robot.
- **Use exact pose to exercise strategy or compare against SelfLocate.**
  Use `use_mujoco_localization:=true`. The bridge uses MuJoCo's team-frame
  ground truth when available and feeds `/<robot>/ground_truth/localization`
  to `brain` as an `Odometer` containing XY and heading. The team-frame pose
  always expresses the attack toward +X. PumaSim's
  `/<robot>/ground_truth/odom` remains the fixed-world reference.
- **GameController.** The demo's ROS GameController node receives UDP
  match packets (default port `3838`) and publishes
  `/robocup/game_controller`. Run or connect the GameController with the
  roster's team IDs. Start in `INITIAL` with the robots outside, then send
  `READY` so they can walk to kickoff positions. At `SET` they should
  be in legal positions and stationary. `PLAY` enables active gameplay.
  [RoboCup HSL Law 7](https://github.com/RoboCup-HumanoidSoccerLeague/HSL-Rules/blob/main/rules/game.tex#L26-L44)
  `--no-gc` is for a session where an existing node supplies
  that common ROS topic. For match launches, only use
  `second_half_swaps_teams:=true` when team order in the incoming packets is
  fixed at halftime. The supervisor already reacts to team order changing.
- **Isolate components.** For the single-robot script, add `--no-vision`,
  `--no-brain`, `--no-gc` or `--no-bridge` as needed. The multi-robot script
  exposes the first three. Use the foreground launch commands below to
  inspect the available arguments and to control the match supervisor.
- **Stop.** Run `./scripts/stop.sh --sim` for a session started by the shell
  script. Stop PumaSim separately with Ctrl+C in its terminal. The shell
  scripts assume one demo session per host. Use a foreground launch and
  Ctrl+C to stop an individual launch.

```bash
ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py --show-args
ros2 launch pumasim_booster_bridge robocup_demo_match.launch.py --show-args

# Single robot with only the bridge.
./scripts/start.sh --sim robot_type:=t1 --no-vision --no-brain --no-gc

# Run vision and brain in the foreground for a single K1.
# Add enable_bridge:=false if a compatible bridge is already running.
ros2 launch pumasim_booster_bridge robocup_demo_robot.launch.py \
  robot_name:=blue_1 robot_type:=k1 player_role:=striker \
  enable_vision:=true enable_brain:=true

# Foreground K1 match with no automatic placement or side changes.
ros2 launch pumasim_booster_bridge robocup_demo_match.launch.py \
  'robots:=blue_1:70:1:striker:left,red_1:71:1:striker:left' robot_type:=k1 \
  enable_match_supervisor:=false use_mujoco_localization:=true

# Inspect one robot in a multi-robot session.
ros2 topic echo /red_1/ground_truth/localization --once
ros2 topic info /red_1/booster/boostercamera/head/rgb --verbose
ros2 topic echo /red_1/cmd_vel --once
```

For single-robot mode, the Booster-facing topics live at their original
root names. Match mode isolates them under `/<robot>/booster` while keeping
the GameController shared. See the [bridge topic guide](src/pumasim_booster_bridge/README.md)
for message types, RPC mappings and QoS. The RGB relay defaults to enabled for
the demo's reliable subscriptions. It preserves capture stamps. Other
consumers that accept best-effort images can subscribe directly to
`/<robot>/camera/image_raw`.

## Install and use Rerun

**Install the C++ SDK** used by `brain`. From `demorobocup_ws`:

```bash
sudo apt install build-essential cmake curl unzip
bash ../beijing/scripts/install_rerun_cpp.sh
./scripts/build.sh --sim
```

The installer uses [Rerun 0.20.3](https://github.com/rerun-io/rerun/releases/tag/0.20.3)
and builds its Arrow dependency. The default prefix is `/opt/rerun_sdk`.
For a different prefix, export `RERUN_SDK_PREFIX` before both installation and
build. The installer also accepts a downloaded C++ SDK zip and optional Arrow
source archive as positional arguments. Check the build output for the
message about Rerun being disabled if discovery fails.

**Open the Viewer** with the same version as the recording SDK. The Viewer
is a separate uv tool. It does not go in the ROS workspace environment.

```bash
uvx --python /usr/bin/python3 --from rerun-sdk==0.20.3 rerun --version
```

**Record a run to disk.** Use the single-robot script, which launches the
native `brain` launch file. With the workspace sourced, find its active config:

```bash
brain_config="$(ros2 pkg prefix --share brain)/config/config_local.yaml"
printf '%s\n' "$brain_config"
```

Create that file or merge this block into its existing `brain_node` parameters.
For source-controlled or shared config changes, edit the source package and
rebuild to update the installed config. Use an absolute directory writable by
the user running `brain`.

```yaml
brain_node:
  ros__parameters:
    rerunLog:
      enable_file: true
      enable_tcp: false
      log_dir: "/tmp/pumanoids-rerun"
```

Start PumaSim and the one-robot demo. Stop the session after recording, then
choose the timestamped `.rrd` under that directory:

```bash
find /tmp/pumanoids-rerun -name '*.rrd'
uvx --python /usr/bin/python3 --from rerun-sdk==0.20.3 rerun /absolute/path/to/recording.rrd
```

For a browser on WSL, add `--serve-web` and use the URL printed by Rerun.
In the Viewer, inspect `field/robot`, `field/ball`, detected field lines and
the image entities along the simulation-time timeline. Recording and viewing
afterward keeps the live Viewer out of the simulation run.

**View live.** Start the pinned Viewer:

```bash
uvx --python /usr/bin/python3 --from rerun-sdk==0.20.3 rerun --serve-web
```

Set `rerunLog.enable_tcp: true` and
`rerunLog.server_ip: "127.0.0.1:9876"` in the local parameters before starting
`brain`. Open the URL printed by the Viewer. The integrated match launch
currently disables Rerun logging for its per-robot `brain` nodes. Use the
single-robot flow above for recordings and live inspection.

## Validation and pending work

With ROS and the workspace sourced, run the bridge tests using uv:

```bash
uv run --no-project --python /usr/bin/python3 --with 'pytest>=7,<8' --with pyyaml \
  pytest -q src/pumasim_booster_bridge/test/test_booster_protocol.py

PUMASIM_ROOT="$(cd ../.. && pwd)/PumaSim" PUMASIM_TEST_ROS2=1 \
MUJOCO_GL=egl PYOPENGL_PLATFORM=egl \
uv run --no-project --python /usr/bin/python3 --with 'pytest>=7,<8' --with pyyaml \
  pytest -q -rs src/pumasim_booster_bridge/test
```

Set `PUMASIM_ROOT` to the actual checkout if the repositories are not
siblings. The ROS integration tests need the compiled demo nodes. One
optional test also needs an installed upstream demo overlay.

- **Pumanoids.** Validate repeated one-robot startup with both localization
  modes and calibrated `CameraInfo`. Verify both players in each 1v1
  with the real GameController, including assignments, side changes and
  recovery after a restart. Requests for kicks and get-up remain refused
  until PumaSim registers the corresponding skill policies.
- **PumaSim.** Implement those skill policies and verify walking handoff,
  field placement and ground-truth frame transitions. See the
  [simulator guide](https://github.com/LIRA-UNAM/PumaSim/blob/full_ros2topic/docs/pumanoids-demo.md).
- **Performance.** Use the current development PC for functional checks.
  Measure full-demo throughput and latency on a host that can sustain the
  chosen scenario. Report render rate, ROS receive rate and simulation
  real-time factor separately. The `fps: 30` setting specifies the capture
  schedule in simulation time.
