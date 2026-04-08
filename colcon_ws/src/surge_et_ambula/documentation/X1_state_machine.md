# The Master Launch

This page provides an overview of the master launch file, which is responsible for running all the necessary nodes for the robot's operation.

In the [launch directory](../../colcon_ws/src/surge_et_ambula/launch) of the `surge_et_ambula` package, you will find three master launch files, one for each robot model:
- `G1_state_machine.launch.py` for the *Unitree G1*.
- `T1_state_machine.launch.py` for the *Booster T1*.
- `K1_state_machine.launch.py` for the *Booster K1*.

## Differences between the three launch files

While they share the majority of the nodes, there's some key differences that allows them to work with the specific hardware of each robot: