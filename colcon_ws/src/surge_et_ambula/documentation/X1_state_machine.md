# The Master Launch

This page provides an overview of the master launch file, which is responsible for running all the necessary nodes for the robot's operation.

In the [launch directory](../../colcon_ws/src/surge_et_ambula/launch) of the `surge_et_ambula` package, you will find three master launch files, one for each robot model:

- `G1_state_machine.launch.py` for the *Unitree G1*.
- `T1_state_machine.launch.py` for the *Booster T1*.
- `K1_state_machine.launch.py` for the *Booster K1*.

## Launch file breakdown

| ROS 2 Element | Description |
| :--- | :--- |
| 🔶 **`X1_state_machine.launch`** | Master launch file. |
| ├── 🟢 [`game_planner`](../../planning/game_planner/documentation/game_planner.md) | General state machine. |
| ├── 🟢 [`position_start`](../../planning/position_start/README.md) | Positioning. |
| ├── 🟢 [`nv12_converter_node`](../../vision/boosterk1_image_proc/README.md) | Only for Booster K1. Video encoder conversion. |
| ├── 🔶 [**x1_twist.launch.py**](../../hardware/documentation/x1_twist.md) | Movement launch file. |
| │&emsp;&emsp;├── 🟢 [twist_to_x1.py](./src/api/client.js) | Robot walking. |
| │&emsp;&emsp;├── 🟢 [pantilt_to_x1.py](./src/api/client.js) | Head movement. |
| │&emsp;&emsp;└── 🟢 [odom_to_tf.py](./src/api/client.js) | Odometry transformation. |
| └── 🔶 [**ball_follower.launch.py**](./src/components) | Ball following launch file. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;├── 🟢 [ball_detector.py](./src/api/client.js) | Ball detection through CV. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;├── 🟢 [ball_follower.py](./src/api/client.js) | Walking towards the ball. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;└── 🟢 [head_ball_follower.py](./src/api/client.js) | Following the ball with the head. |


## Differences between the three launch files

While they share the majority of the nodes, there's some key differences that allows them to work with the specific hardware of each robot:

### Movement nodes

For high-level movement control, there's specific nodes for each robot model.

| Robot Model | Movement Nodes |
|---|---|
| Unitree G1 | `twist_to_g1`, `pantilt_to_g1`, `odom_to_tf` |
| Booster T1 | [`twist_to_t1`](../../hardware/booster/twist_to_t1/twist_to_t1/twist_to_t1.py), [`pantilt_to_t1`](../../hardware/booster/twist_to_t1/twist_to_t1/pantilt_to_t1.py), [`odom_to_tf`](../../hardware/booster/twist_to_t1/twist_to_t1/odom_to_tf.py) |
| Booster K1 | [`twist_to_k1`](../../hardware/booster_k1/twist_to_k1/twist_to_k1/twist_to_k1.py), [`pantilt_to_k1`](../../hardware/booster_k1/twist_to_k1/twist_to_k1/pantilt_to_k1.py), [`odom_to_tf`](../../hardware/booster_k1/twist_to_k1/twist_to_k1/odom_to_tf.py) |

> The source files for the Unitree G1 nodes aren't in this repository, but in the `/home/unitree` directory on the robot itself.

### Video encoding conversion

The native topics publishing the camera feed on the Booster K1 are encoded in `nv12`, which is not compatible our image processing nodes. To solve this, the [`nv12_converter_node`](../../vision/boosterk1_image_proc/src/nv12_converter_node.cpp) is included in the `K1_state_machine.launch.py` to convert the video encoding to `bgr8`.

| Robot Model | Node |
|---|---|
| Booster K1 | [`nv12_converter_node`](../../vision/boosterk1_image_proc/src/nv12_converter_node.cpp) |

