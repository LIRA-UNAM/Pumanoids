"""Reference rollout of a K1 velocity ONNX policy without ROS (ground truth for k1_sim2sim.py).

Same scene (export_k1_sim_model.py) and the same scenario as k1_sim2sim.py, but the
observation is built here exactly as in the mjlab task (base_ang_vel = IMU gyro,
projected_gravity from the trunk orientation, joint_pos - default and joint_vel of
the 20 action joints, last action, command) and the actuators are driven
directly at the training gains. If a command is tracked here but not through
k1_ppowalk, the deploy path is at fault; if it fails here too, it is the policy.

  ~/.venvs/k1sim/bin/pip install onnxruntime   # or use mjlab's .venv
  python k1_offline_rollout.py --model ~/k1_ppowalk_sim/k1_flat_scene.mjb --onnx policy.onnx
"""

import argparse
import math

import mujoco
import numpy as np
import onnxruntime as ort

from k1_sim2sim import JOINTS, SCENARIO, SETTLE_S, quat_to_rpy

ACTION = JOINTS[2:]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--onnx", required=True)
    args = ap.parse_args()
    m = mujoco.MjModel.from_binary_path(args.model)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    meta = sess.get_modelmeta().custom_metadata_map
    scale = np.array([float(v) for v in meta["action_scale"].split(",")])
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "robot/" + n)
    aid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "robot/" + n)
    qa = np.array([m.jnt_qposadr[jid(n)] for n in ACTION])
    va = np.array([m.jnt_dofadr[jid(n)] for n in ACTION])
    act = np.array([aid(n) for n in ACTION])
    default = d.qpos[qa].copy()
    # Exact training scales (metadata is rounded to 3 decimals).
    exact = {0.35: 0.35, 0.213: 0.2125, 0.237: 0.2375, 0.12: 0.1196875, 0.383: 0.383}
    scale = np.array([exact.get(round(s, 3), s) for s in scale])
    head = np.array([aid(n) for n in JOINTS[:2]])
    d.ctrl[head] = 0.0
    gyro = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "robot/imu_ang_vel")]
    last = np.zeros(20, dtype=np.float32)
    d.ctrl[act] = default
    rows = []
    t = 0.0
    for name, dur, vx, vy, wz in SCENARIO:
        seg_t = 0.0
        while seg_t < dur:
            w, x, y, z = d.qpos[3:7]
            # projected gravity = R^T (0, 0, -1)
            g = -np.array([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])
            obs = np.concatenate([d.sensordata[gyro:gyro + 3], g, d.qpos[qa] - default,
                                  d.qvel[va], last, [vx, vy, wz]]).astype(np.float32)
            last = sess.run(None, {"obs": obs[None]})[0][0]
            d.ctrl[act] = default + scale * last
            for _ in range(4):
                mujoco.mj_step(m, d)
            seg_t += 0.02
            t += 0.02
            roll, pitch, yaw = quat_to_rpy(*d.qpos[3:7])
            v = d.qvel[:3]
            c, s = math.cos(yaw), math.sin(yaw)
            tilt = math.degrees(math.acos(math.cos(roll) * math.cos(pitch)))
            rows.append((name, seg_t, d.qpos[2], c * v[0] + s * v[1], -s * v[0] + c * v[1],
                         d.qvel[5], tilt))
            if d.qpos[2] < 0.3:
                print(f"FALL in {name} at {t:.1f} s")
                return
    print("segment     cmd(vx,vy,wz)        mean(vx,vy,wz)          min h   max tilt")
    for name, _, vx, vy, wz in SCENARIO:
        a = np.array([r[2:] for r in rows if r[0] == name and r[1] >= SETTLE_S])
        print(f"{name:8s} ({vx:+.2f},{vy:+.2f},{wz:+.2f})  ({a[:, 1].mean():+.2f},"
              f"{a[:, 2].mean():+.2f},{a[:, 3].mean():+.2f})   {a[:, 0].min():.3f}   "
              f"{a[:, 4].max():5.1f}")


if __name__ == "__main__":
    main()
