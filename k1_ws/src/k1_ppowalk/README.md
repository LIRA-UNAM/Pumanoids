# k1_ppowalk

Runs velocity-tracking (walking) policies trained with mjlab-biped-playground
(`Pumas-Velocity-{Flat,Rough}-Booster-K1-PPO[-DA]`, IMU-only actor) on the
Booster K1 through the low-level interface: it reads `booster_interface/LowState`
and publishes `booster_interface/LowCmd` itself. Port of `t1_ws/src/t1_ppowalk`;
the web GUI is `k1_ppowalk_gui` (http://<robot-ip>:8085). Controls, remote,
model switching, timing and the DDS profile work exactly as in the T1 README;
this file lists what differs.

```bash
make k1walk                                    # no model loaded: pick one in the GUI
make k1walk K1_MODELS_DIR=/abs/folder
make k1walk K1_WALK_POLICY=/abs/path/model.onnx K1_WALK_ARGS="enable_lowcmd:=true"
```

## What the node reproduces from training

The K1 task trains the **parallel-ankle model** (`k1_parallel.xml`): each ankle
is driven by two cranks (`*_Ankle_A`, `*_Ankle_B`) through rods, and ankle
pitch/roll are passive. The policy observes and commands the **crank angles**.
All values come from `tasks/velocity/config/k1/env_cfgs.py`,
`asset_zoo/robots/k1/k1_parallel_constants.py` / `k1_constants.py` and were
checked against the ONNX metadata (`config/k1_ppowalk_policy.yaml`):

* 22 motors: head (2), left arm (4), right arm (4), per leg hip pitch/roll/yaw,
  knee, crank A, crank B. No waist.
* Observation (69): `base_ang_vel` (3), `projected_gravity` (3), `joint_pos -
  default` (20), `joint_vel` (20), last raw action (20), command (3). The 20
  observed joints = the action joints = everything but the head
  (`observation_joint_names`).
* Action (20): `target = default + action_scale * a`, `action_scale = 0.25 *
  effort / kp` (arms 0.35, hip pitch 0.2125, hip roll 0.2375, hip yaw 0.1197,
  knee 0.35, cranks 0.383). Policy 50 Hz, LowCmd 200 Hz.
* Gains: hips / knee 80 / 4, cranks 25 / 1, arms 10 / 1, head 4 / 0.25 (held at 0).
* HOME_KEYFRAME: hip pitch −0.4, knee 0.8, cranks A 0.334363 / B 0.317093
  (= ankle pitch −0.4), shoulder roll ∓1.4, elbow yaw ∓0.4.
* Velocity limits (conservative; trained up to vx −2..3, vy ±0.5, wz ±0.7):
  vx ±1.0, vy ±0.5, wz ±0.7.

## Ankles: PARALLEL mode + linkage check

Unlike the T1 node (SERIAL, firmware maps pitch/roll), this node sends **LowCmd
`cmd_type` PARALLEL** (enforced) and reads `motor_state_parallel`, so the crank
motors get exactly the training targets and gains. That only works if the
firmware's crank angles use the training model's convention (zero, sign, A/B
order). The node checks it on every LowState: the firmware's serial ankle
pitch/roll (`motor_state_serial`, crank slots) are mapped through the training
linkage's closed-form IK (`include/k1_ppowalk/ankle_linkage.hpp`, unit-tested
against the keyframe crank angles) and compared with the measured cranks. The
GUI shows the result under "Ankle linkage check"; with a mismatch above
`linkage_tolerance` (0.05 rad) Start is refused (`require_linkage_check`).

## To confirm on the robot

* The linkage check reads "cranks match sim" with the robot hanging and damped,
  at several ankle poses (move the feet by hand). A mismatch means the crank
  convention differs (offset, sign or A/B swapped): do not walk; the fix is a
  per-crank sign/offset mapping in the node.
* Motor order of LowState / LowCmd = `joint_names` (head, left arm, right arm,
  left leg, right leg); `/low_state`, `/joint_ctrl` topics.
* `custom_mode` (3) / `prepare_mode` (1) and the remote's `back` / `start` fields.
* While hanging, Home must hold the cranks at 0.334 / 0.317 without oscillating.

## Sim2sim test (laptop)

`test/sim/` runs the node against the exact mjlab training scene in MuJoCo:

```bash
# once: compiled training scene + a venv with MuJoCo next to ROS (Python 3.12)
cd ~/mjlab-biped-playground && .venv/bin/python \
    ~/Pumanoids/k1_ws/src/k1_ppowalk/test/sim/export_k1_sim_model.py ~/k1_ppowalk_sim/k1_flat_scene.mjb
uv venv --python /usr/bin/python3.12 --system-site-packages ~/.venvs/k1sim
VIRTUAL_ENV=~/.venvs/k1sim uv pip install "mujoco==3.11.0" onnxruntime

# each run, on an isolated ROS domain (never on the robot's network domain)
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash; source colcon_ws/install/setup.bash; source k1_ws/install/setup.bash
$(ros2 pkg prefix k1_ppowalk)/lib/k1_ppowalk/k1_ppowalk_policy_node --ros-args \
    --params-file $(ros2 pkg prefix k1_ppowalk)/share/k1_ppowalk/config/k1_ppowalk_policy.yaml \
    -p policy_path:=$HOME/k1_ppowalk_models/<model>.onnx -p enable_lowcmd:=true \
    -p use_remote:=false -p use_cmd_vel:=true &
~/.venvs/k1sim/bin/python k1_ws/src/k1_ppowalk/test/sim/k1_sim2sim.py \
    --model ~/k1_ppowalk_sim/k1_flat_scene.mjb --auto --csv /tmp/k1_sim.csv
```

`k1_sim2sim.py` publishes LowState (IMU, parallel and serial motor views) at
200 Hz, applies LowCmd through the training actuators, holds the trunk until the
policy starts, calls Home then Start, runs a `/cmd_vel` scenario and prints the
tracking per segment (exit 0 = no fall). Without `--auto` it just simulates (use
the GUI). `k1_offline_rollout.py` runs the same scenario without ROS (observation
built directly) as the reference the ROS run must match. Not simulated: domain
randomization, actuator delay, sensor noise.

## Export a policy

```bash
cd ~/mjlab-biped-playground
uv run python export_policy.py Pumas-Velocity-Flat-Booster-K1-PPO-DA \
    --wandb-run-path Pumanoids/k1_velocity/<run> \
    --export-dir ~/k1_ppowalk_models --filename k1_velocity_flat_ppo_da_<run>.onnx
```
