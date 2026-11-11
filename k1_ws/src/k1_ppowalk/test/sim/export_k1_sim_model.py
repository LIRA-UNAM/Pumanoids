"""Save the compiled mjlab K1 training scene (flat) as a MuJoCo binary for k1_sim2sim.py.

Run with mjlab-biped-playground's interpreter (same MuJoCo version as the
harness: the .mjb format is version specific):

  cd ~/mjlab-biped-playground
  .venv/bin/python ~/Pumanoids/k1_ws/src/k1_ppowalk/test/sim/export_k1_sim_model.py \
      ~/k1_ppowalk_sim/k1_flat_scene.mjb

The model is exactly the one the policy was trained on: k1_parallel.xml with
the parallel ankle linkage, mjlab's implicit position actuators (training kp/kd,
effort limits as force ranges, armature), the flat terrain plane and the task's
MuJoCo options (dt 0.005, implicitfast, newton, pyramidal cone). Domain
randomization, actuator delay and observation noise are not reproduced.
"""

import sys
from pathlib import Path

import mujoco
from mjlab.scene import Scene

from playground.tasks.velocity.config.k1.env_cfgs import k1_flat_env_cfg


def main() -> None:
  out = Path(sys.argv[1] if len(sys.argv) > 1 else "~/k1_ppowalk_sim/k1_flat_scene.mjb")
  out = out.expanduser()
  out.parent.mkdir(parents=True, exist_ok=True)
  cfg = k1_flat_env_cfg(play=True)
  cfg.scene.num_envs = 1
  model = Scene(cfg.scene, device="cpu").compile()
  cfg.sim.mujoco.apply(model)
  mujoco.mj_saveModel(model, str(out), None)
  print(f"saved {out} (MuJoCo {mujoco.__version__}, dt {model.opt.timestep}, "
        f"{model.nu} actuators, {model.nkey} keyframe)")


if __name__ == "__main__":
  main()
