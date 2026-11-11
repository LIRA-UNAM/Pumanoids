"""Sim2sim harness for k1_ppowalk: a simulated K1 behind the Booster low-level topics.

Runs the compiled mjlab K1 training scene (export_k1_sim_model.py) in real time
and stands in for the robot's firmware:

* publishes booster_interface/LowState on /low_state every sim step (200 Hz):
  IMU rpy / gyro / acc from the trunk IMU site, motor_state_parallel = the 22
  actuated joints (ankle slots = crank angles), motor_state_serial = the same
  with the ankle slots replaced by the (passive) ankle pitch/roll, as the
  Booster firmware reports them (so the node's linkage check runs for real);
* applies booster_interface/LowCmd from /joint_ctrl (PARALLEL only) through the
  training actuators: per motor kp -> gainprm / biasprm, kd -> biasprm, q ->
  ctrl (implicit PD, effort limits as force ranges, as in training).

The trunk is held at the HOME keyframe (feet on the ground) until the policy
node reports RAMPING / RUNNING, like a robot lowered onto the floor before Start.

--auto runs a test scenario: waits for the node (enable_lowcmd, use_cmd_vel,
linkage check passed), calls ~/home, then ~/start once HOLDING, publishes a /cmd_vel schedule, logs to CSV and prints a
per-segment tracking summary; exit code 0 = no fall.

  source /opt/ros/jazzy/setup.bash; source ~/Pumanoids/colcon_ws/install/setup.bash
  ~/.venvs/k1sim/bin/python k1_sim2sim.py --model ~/k1_ppowalk_sim/k1_flat_scene.mjb --auto \
      --csv /tmp/k1_sim.csv
"""

import argparse
import csv
import json
import math
import sys
import threading
import time

import mujoco
import numpy as np
import rclpy
from booster_interface.msg import LowCmd, LowState, MotorState
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

JOINTS = [
    "Head_Yaw", "Head_Pitch",
    "Left_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
    "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Knee_Pitch", "Left_Ankle_A",
    "Left_Ankle_B",
    "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch", "Right_Ankle_A",
    "Right_Ankle_B",
]
# Serial view: crank slots report ankle pitch (A slot) / roll (B slot).
SERIAL = {"Left_Ankle_A": "Left_Ankle_Pitch", "Left_Ankle_B": "Left_Ankle_Roll",
          "Right_Ankle_A": "Right_Ankle_Pitch", "Right_Ankle_B": "Right_Ankle_Roll"}

# (duration s, vx, vy, wz); the first segment is the stand test.
SCENARIO = [
    ("stand", 10.0, 0.0, 0.0, 0.0),
    ("vx+0.5", 10.0, 0.5, 0.0, 0.0),
    ("vx-0.3", 6.0, -0.3, 0.0, 0.0),
    ("vy+0.3", 8.0, 0.0, 0.3, 0.0),
    ("wz+0.5", 8.0, 0.0, 0.0, 0.5),
    ("vx1.0", 8.0, 1.0, 0.0, 0.0),
    ("stop", 5.0, 0.0, 0.0, 0.0),
]
SETTLE_S = 3.0  # skipped at the start of every segment when averaging


def quat_to_rpy(w, x, y, z):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


class K1Sim(Node):
    def __init__(self, args):
        super().__init__("k1_sim2sim")
        self.args = args
        self.m = mujoco.MjModel.from_binary_path(args.model)
        self.d = mujoco.MjData(self.m)
        assert abs(self.m.opt.timestep - 0.005) < 1e-9, self.m.opt.timestep
        jid = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "robot/" + n)
        aid = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "robot/" + n)
        self.qadr = np.array([self.m.jnt_qposadr[jid(n)] for n in JOINTS])
        self.vadr = np.array([self.m.jnt_dofadr[jid(n)] for n in JOINTS])
        sj = [SERIAL.get(n, n) for n in JOINTS]
        self.sqadr = np.array([self.m.jnt_qposadr[jid(n)] for n in sj])
        self.svadr = np.array([self.m.jnt_dofadr[jid(n)] for n in sj])
        self.act = np.array([aid(n) for n in JOINTS])
        assert (self.act >= 0).all() and (self.qadr >= 0).all()
        sid = lambda n: self.m.sensor_adr[
            mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, "robot/" + n)]
        self.s_quat, self.s_gyro, self.s_acc = sid("orientation"), sid("imu_ang_vel"), sid(
            "imu_lin_acc")
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        self.key_base = self.d.qpos[:7].copy()
        self.set_gains(np.zeros(22), np.zeros(22), self.d.qpos[self.qadr].copy())
        mujoco.mj_forward(self.m, self.d)

        self.lock = threading.Lock()
        self.cmd = None
        self.cmd_count = 0
        self.held = True
        self.status = {}
        self.sim_time = 0.0
        self.pub = self.create_publisher(LowState, "/low_state", 10)
        self.create_subscription(LowCmd, "/joint_ctrl", self.on_cmd, 10)
        self.create_subscription(String, f"{args.node}/status", self.on_status, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start_cli = self.create_client(Trigger, f"{args.node}/start")
        self.home_cli = self.create_client(Trigger, f"{args.node}/home")
        self.msg = LowState()
        self.msg.motor_state_parallel = [MotorState() for _ in JOINTS]
        self.msg.motor_state_serial = [MotorState() for _ in JOINTS]

    def set_gains(self, kp, kd, q):
        a = self.act
        self.m.actuator_gainprm[a, 0] = kp
        self.m.actuator_biasprm[a, 1] = -kp
        self.m.actuator_biasprm[a, 2] = -kd
        self.d.ctrl[a] = q

    def on_cmd(self, msg):
        if msg.cmd_type != LowCmd.CMD_TYPE_PARALLEL:
            self.get_logger().error("LowCmd not PARALLEL: ignored", throttle_duration_sec=2.0)
            return
        if len(msg.motor_cmd) < 22:
            return
        mc = msg.motor_cmd[:22]
        with self.lock:
            self.cmd = (np.array([c.kp for c in mc]), np.array([c.kd for c in mc]),
                        np.array([c.q for c in mc]))
            self.cmd_count += 1

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            return
        if self.held and self.status.get("mode") in ("RAMPING", "RUNNING"):
            self.held = False
            self.get_logger().info("policy started: trunk released")

    def step(self):
        with self.lock:
            cmd = self.cmd
        if cmd is not None:
            self.set_gains(*cmd)
        mujoco.mj_step(self.m, self.d)
        if self.held:
            self.d.qpos[:7] = self.key_base
            self.d.qvel[:6] = 0.0
            mujoco.mj_forward(self.m, self.d)
        self.sim_time += self.m.opt.timestep
        self.publish()

    def publish(self):
        sd = self.d.sensordata
        w, x, y, z = sd[self.s_quat:self.s_quat + 4]
        imu = self.msg.imu_state
        imu.rpy = [float(v) for v in quat_to_rpy(w, x, y, z)]
        imu.gyro = [float(v) for v in sd[self.s_gyro:self.s_gyro + 3]]
        imu.acc = [float(v) for v in sd[self.s_acc:self.s_acc + 3]]
        q, dq = self.d.qpos[self.qadr], self.d.qvel[self.vadr]
        sq, sdq = self.d.qpos[self.sqadr], self.d.qvel[self.svadr]
        tau = self.d.actuator_force[self.act]
        for i in range(22):
            p, s = self.msg.motor_state_parallel[i], self.msg.motor_state_serial[i]
            p.q, p.dq, p.tau_est = float(q[i]), float(dq[i]), float(tau[i])
            s.q, s.dq, s.tau_est = float(sq[i]), float(sdq[i]), float(tau[i])
        self.pub.publish(self.msg)

    def base_state(self):
        """Height, heading-frame linear velocity (vx, vy), yaw rate, tilt (deg)."""
        qw, qx, qy, qz = self.d.qpos[3:7]
        roll, pitch, yaw = quat_to_rpy(qw, qx, qy, qz)
        v = self.d.qvel[:3]
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = c * v[0] + s * v[1], -s * v[0] + c * v[1]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
        return float(self.d.qpos[2]), vx, vy, float(self.d.qvel[5]), tilt


def call(client, name):
    fut = client.call_async(Trigger.Request())
    fut.add_done_callback(lambda f: client.handle.service_name and print(
        f"[k1_sim2sim] {name}: {f.result().success} {f.result().message}", flush=True))


def run(sim, args, ex):
    dt = sim.m.opt.timestep
    log, rows = None, []
    if args.csv:
        log = csv.writer(open(args.csv, "w", newline=""))
        log.writerow(["t", "segment", "cmd_vx", "cmd_vy", "cmd_wz", "height", "vx", "vy", "wz",
                      "tilt_deg", "mode"])
    phase = "wait" if args.auto else "manual"
    seg_i, seg_t0, started_at, home_called, last_call = 0, 0.0, None, False, -10.0
    fell = False
    t0 = time.perf_counter()
    n = 0
    max_gap, last_wall = 0.0, time.perf_counter()
    while rclpy.ok():
        sim.step()
        n += 1
        # Single thread: serve LowCmd / status / service callbacks between steps.
        for _ in range(8):
            ex.spin_once(timeout_sec=0.0)
        now = time.perf_counter()
        max_gap, last_wall = max(max_gap, now - last_wall), now
        st = sim.status
        if phase == "wait":
            ready = (st.get("policy_loaded") and st.get("enable_lowcmd") and st.get("data_fresh")
                     and "match" in st.get("ankle_space", ""))
            # Home, then Start once HOLDING; retried every 2 s if refused.
            mode = st.get("mode")
            if sim.sim_time > 2.0 and ready and mode == "IDLE" and sim.sim_time - last_call > 2.0:
                last_call = sim.sim_time
                if not home_called:
                    sim.get_logger().info(f"linkage: {st.get('ankle_space')}")
                home_called = True
                call(sim.home_cli, "home")
            if mode == "HOLDING" and sim.sim_time - last_call > 2.0:
                last_call = sim.sim_time
                call(sim.start_cli, "start")
            if st.get("mode") == "RUNNING":
                phase, seg_t0, started_at = "run", sim.sim_time, sim.sim_time
            if sim.sim_time > args.timeout:
                sim.get_logger().error(f"node not ready after {args.timeout} s: {st}")
                return 2
        cmd = (0.0, 0.0, 0.0)
        name = phase
        if phase == "run":
            name, dur, *cmd = SCENARIO[seg_i]
            if sim.sim_time - seg_t0 >= dur:
                seg_i += 1
                seg_t0 = sim.sim_time
                if seg_i == len(SCENARIO):
                    break
                name, dur, *cmd = SCENARIO[seg_i]
            if n % 4 == 0:
                tw = Twist()
                tw.linear.x, tw.linear.y, tw.angular.z = cmd
                sim.cmd_vel_pub.publish(tw)
        h, vx, vy, wz, tilt = sim.base_state()
        if phase == "run":
            rows.append((name, sim.sim_time - seg_t0, *cmd, h, vx, vy, wz, tilt))
            if h < 0.3 or tilt > 60:
                fell = True
                sim.get_logger().error(f"FALL in segment {name} at t={sim.sim_time:.1f}s")
                break
        if log and n % 4 == 0:
            log.writerow([f"{sim.sim_time:.3f}", name, *cmd, f"{h:.4f}", f"{vx:.4f}",
                          f"{vy:.4f}", f"{wz:.4f}", f"{tilt:.2f}", st.get("mode", "")])
        # Real time: the node's timers run on the wall clock.
        # No catch-up bursts: a late step drops the debt (sim time slows down).
        lag = t0 + n * dt - time.perf_counter()
        if lag > 0:
            time.sleep(lag)
        elif lag < -0.02:
            t0 -= lag
        if n % 2000 == 0:
            if max_gap > 0.05:
                sim.get_logger().warn(f"max step gap {max_gap * 1e3:.0f} ms in the last 10 s")
            max_gap = 0.0
    if not args.auto:
        return 0
    print("\nsegment     cmd(vx,vy,wz)        mean(vx,vy,wz)          min h   max tilt")
    for seg in SCENARIO:
        sel = [r for r in rows if r[0] == seg[0] and r[1] >= SETTLE_S]
        if not sel:
            continue
        a = np.array([r[2:] for r in sel], dtype=float)
        print(f"{seg[0]:8s} ({seg[2]:+.2f},{seg[3]:+.2f},{seg[4]:+.2f})  "
              f"({a[:, 4].mean():+.2f},{a[:, 5].mean():+.2f},{a[:, 6].mean():+.2f})   "
              f"{a[:, 3].min():.3f}   {a[:, 7].max():5.1f}")
    st = sim.status.get("control", {})
    print(f"LowCmd received: {sim.cmd_count}, node control: {json.dumps(st)}")
    print("RESULT:", "FELL" if fell else "OK (no fall)")
    return 1 if fell else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True, help="compiled scene (.mjb) from export_k1_sim_model.py")
    ap.add_argument("--node", default="/k1_ppowalk_policy_node")
    ap.add_argument("--auto", action="store_true", help="run the test scenario and exit")
    ap.add_argument("--csv", default="")
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()
    rclpy.init()
    sim = K1Sim(args)
    ex = SingleThreadedExecutor()
    ex.add_node(sim)
    try:
        code = run(sim, args, ex)
    except KeyboardInterrupt:
        code = 0
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
