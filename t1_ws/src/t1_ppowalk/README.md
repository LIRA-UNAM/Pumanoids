# t1_ppowalk

Runs velocity-tracking (walking) policies trained with
mjlab-biped-playground (`Pumas-Velocity-{Flat,Rough}-Booster-T1-PPO[-DA]`,
IMU-only actor) on the Booster T1, through the low-level interface: it reads
`booster_interface/LowState` and publishes `booster_interface/LowCmd` itself (no
separate bridge). Port of G1-Software's `ppowalk` / `ppowalk_gui`; the web GUI is
`t1_ppowalk_gui` (http://<robot-ip>:8084).

```bash
make t1walk                                    # no model loaded: pick one in the GUI
make t1walk T1_MODELS_DIR=/abs/folder          # another models folder
make t1walk T1_WALK_POLICY=/abs/path/model.onnx T1_WALK_ARGS="enable_lowcmd:=true"
```

`booster_interface` comes from `colcon_ws`: build and source it before `t1_ws`
(`make t1walk` does both). Every `*.onnx` in the models folder (`models_dir`,
default `~/t1_ppowalk_models`; rescanned every second, so copied models show up
without a restart) is offered in the GUI's Model dropdown. Choosing one loads it
while the policy is stopped; the next Start uses it. While the policy runs,
choosing one switches to it live (no Home / Start), but only at a standstill:
zero requested velocity, body angular velocity <= `switch_max_ang_vel`
(0.3 rad/s) and leg joint velocities <= `switch_max_joint_vel` (0.5 rad/s)
for `switch_settle_s` (0.5 s). The new policy blends in over `start_ramp_s`.
Otherwise the dropdown shows why and the node refuses the `policy_path`
change. A model that is not 76 -> 21
is rejected and the current one stays loaded; a model whose mjlab metadata
(joint names, default pose, action scale, kp, kd, observation terms) disagrees
with the parameters is loaded with a warning in the GUI ("Model metadata check").

## What the node reproduces from training

All values come from the task code (`tasks/velocity/config/t1/env_cfgs.py`,
`asset_zoo/robots/booster_t1/t1_constants.py`, `t1.xml`) and live in
`config/t1_ppowalk_policy.yaml`:

* Observation (76): `base_ang_vel` (3, trunk gyro), `projected_gravity` (3, from
  the IMU rpy), `joint_pos - default` (23), `joint_vel` (23), last raw action
  (21), velocity command `(vx, vy, wz)` (3). No scaling, clipping or history.
* Joint order: natural `t1.xml` order = T1 serial motor order (neck, left arm,
  right arm, waist, left leg, right leg).
* Action (21): every joint but the neck, in natural order (not the order of the
  patterns in `env_cfgs.py`); `target = default_joint_pos + action_scale * a`
  with `action_scale = 0.25 * effort / kp`. Policy at 50 Hz (dt 0.005 x 4).
* LowCmd at 200 Hz (= 1 / sim dt), serial, `q = target`, `dq = tau = 0`, training
  gains (5 Hz, damping ratio 2): kp neck 1.7765, arm 27.8844, waist / hip
  roll / hip yaw 47.189, hip pitch 51.7076, knee 62.7719, ankle 33.5124.
  Targets are clamped to the hard `t1.xml` ranges. The simulation caps each
  torque at the effort limit (7-65 Nm); LowCmd position mode cannot, so the
  limits are only shown in the GUI.
* The neck is held at 0 (its own PD holds HOME_KEYFRAME in training). The waist
  can never be dropped from `command_joints`.
* Damping (kp = 0, kd = `damping_kd`, q = measured): every joint while IDLE or
  without fresh LowState, non-commanded joints, and motors reporting error flags
  (`MotorState.reserve[0] != 0`, latched for the run, observation pinned to the
  hold target). Start is refused while a leg motor reports errors.
* Velocity limits = the final curriculum stage: vx ±1.5, vy ±0.7, wz ±0.7.

## Controls

* **Custom mode:** "Enter Custom mode" in the GUI (or `~/enter_custom_mode`)
  sends ChangeMode(`custom_mode` = 3) through `/booster_rpc_service`; "Exit to
  Prepare" sends ChangeMode(`prepare_mode` = 1). Nothing switches automatically.
* **Motor output:** LowCmd is only published with `enable_lowcmd` (GUI toggle or
  launch argument). While IDLE every joint is damped.
* **Upright gate:** Start is refused unless the IMU tilt is at most
  `max_start_tilt_deg` (20 deg).
* **Home** (GUI or BACK (−) on the remote): slow cosine move (peak
  `home_max_vel`, at least `home_min_duration_s`) to `home_pose` = HOME_KEYFRAME,
  then hold. Stops a running policy.
* **Start** (GUI or START (+) on the remote): blends from the current pose into
  the policy targets over `start_ramp_s`, then runs the policy.
* **Stop:** holds the current pose (stays stiff). **Release:** damps everything
  (a standing robot collapses).
* **Velocity command:** one source at a time (GUI "Command source"): the remote
  sticks (default), the GUI joysticks (left = vx/vy, right = wz; releasing a
  stick sends 0), or `geometry_msgs/Twist` on `/cmd_vel`.
  Calibration offsets `cmd_{vx,vy,wz}_offset` cancel drift.
* If LowState goes stale while running, the node holds the last pose (and the
  LowCmd falls back to damping). There is no software e-stop: keep the Booster
  app / remote damping at hand.

## Remote controller

`booster_interface/RemoteControllerState` on `/remote_controller_state`:

* **Sticks** (`use_remote`): left stick front/back = vx, left/right = vy, right
  stick left/right = wz; full deflection = the velocity limits, inside
  `remote_deadzone` exactly 0. Falls back to 0 if the remote stops for
  `remote_timeout_s`.
* **START (+)** (`remote_start`): Start, with every GUI check.
* **BACK (−)** (`remote_home`): Home.
* Buttons act on the press only (holding does not repeat). Refused presses are
  logged and shown in the GUI ("Last remote event").

Typical run: hang the robot, Enter Custom mode, enable motor output, BACK (−) to
move home, lower it until it stands, START (+), steer with the sticks.

## Ankles (parallel linkage)

Each T1 ankle is driven by two crank motors (SDK `kCrankUp/Down{Left,Right}`,
slots 15/16 and 21/22) through a parallel linkage, while the policy (trained on
`t1.xml`) uses ankle pitch/roll joints. No transform is needed in this node: with
`cmd_type` SERIAL (enforced; the node refuses PARALLEL) the firmware maps the
pitch/roll targets onto the cranks, and `LowState.motor_state_serial` reports
pitch/roll (`motor_state_parallel` holds the crank angles; every other joint is
identical in both arrays). Checked on the robot: serial left ankle −0.131 / +0.019
vs cranks +0.142 / +0.160. On the first LowState the node verifies that the
serial and parallel ankle entries differ and shows the result in the GUI
("Ankle space"); "UNVERIFIED" means the firmware did not convert them: do not walk.

## To confirm on the robot

* `low_cmd_topic` (`/joint_ctrl` = SDK `rt/joint_ctrl`) and `/low_state` exist on
  the robot. Still to confirm: `custom_mode` (3) and that BACK / START arrive as
  the `back` / `start` fields (check the GUI's live remote panel before walking).
* Ankle signs, robot damped and hanging: toes down -> ankle pitch increases,
  left (+y) edge of the foot up -> ankle roll increases (`t1.xml` axes).
* SERIAL-mode gains: the firmware's mapping of the ankle kp/kd onto the cranks is
  not documented. While hanging, Home must hold the ankles at pitch −0.2 / roll 0
  without going limp or oscillating.

## Control loop timing

Two causes of LowCmd gaps (jitter on the robot) are handled at the ROS 2 level,
without kernel changes:

* **Executor isolation.** The control loop (LowState subscription + one
  `command_rate_hz` timer that runs the policy every
  `command_rate_hz / policy_rate_hz` ticks, so that ratio must be an integer) runs
  in its own callback group on a dedicated executor thread. Parameter services
  (`ros2 param`, tab completion), parameter sets from the GUI (incl. loading a
  model), the GUI services, remote / `cmd_vel`, `~/status` and the model scan run
  on the main executor. They share state under one mutex that is only held for
  short sections.
* **DDS shared memory.** With Fast DDS's shared-memory transport, every short-lived
  participant (each `ros2 param ...` / `ros2 topic ...` call, tab completion
  included) can block all writers of a node for ~330 ms: the discovery thread
  holds the participant's send mutex while `SharedMemTransport::find_port()`
  retries to open that process's port (found with gdb: LowCmd `publish()` waiting
  in `RTPSWriter::send_nts`). The launch file therefore gives the policy node the
  UDPv4-only profile `config/fastdds_udp_only.xml` (`dds_profile:=none` reverts to
  the Fast DDS defaults). To also keep the CLI from stalling the robot's own
  processes, use the same profile in shells where you run `ros2` commands on the
  robot:

  ```bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=$(ros2 pkg prefix t1_ppowalk)/share/t1_ppowalk/config/fastdds_udp_only.xml
  ```

  The profile is ignored with `ROS_LOCALHOST_ONLY=1` or
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (rmw_fastrtps then sets the transports
  itself).

`~/status` reports the loop timing in `control`: `max_jitter_ms` (max |tick period
− nominal|), `max_exec_ms` (tick duration) and `max_lock_wait_ms`, each since the
previous status message, plus `late_ticks` (ticks more than 1 ms late since start).
The loop still runs at normal OS priority, so a heavily loaded CPU can delay it.

## Export a policy

In `mjlab-biped-playground` (task ids start with `Pumas-`, not `Mjlab-`):

```bash
uv run python export_policy.py Pumas-Velocity-Flat-Booster-T1-PPO-DA \
    --wandb-run-path Pumanoids/t1_velocity/<run> \
    --export-dir ~/t1_ppowalk_models --filename t1_velocity_flat_ppo_da_<run>.onnx
scp ~/t1_ppowalk_models/t1_velocity_flat_ppo_da_<run>.onnx t1:t1_ppowalk_models/
```
