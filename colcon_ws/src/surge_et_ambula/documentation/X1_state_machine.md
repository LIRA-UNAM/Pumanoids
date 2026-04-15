# 🌕 The Master Launch

This page provides an overview of the master launch file, which is responsible for running all the necessary nodes for the robot's operation.

The source files can be found in the [launch directory](/colcon_ws/src/surge_et_ambula/launch/) of the `surge_et_ambula` package. There is one launch file for each robot model:

- `G1_state_machine.launch.py` for the *Unitree G1*.
- `T1_state_machine.launch.py` for the *Booster T1*.
- `K1_state_machine.launch.py` for the *Booster K1*.

## ↪️ Launch file breakdown

Here you can visualize what nodes are launched and what's their role. For more details on each node, click on their name to go to their documentation.

| ROS 2 Element | Description |
| :--- | :--- |
| 🔶 **`X1_state_machine.launch.py`** | Master launch file. |
| ├── 🟢 [`game_planner`](/colcon_ws/src/planning/game_planner/documentation/game_planner.md) | General state machine. |
| ├── 🟢 [`position_start`](/colcon_ws/src/planning/position_start/README.md) | Positioning at startup. |
| ├── 🟢 [`nv12_converter_node`](/colcon_ws/src/vision/boosterk1_image_proc/README.md) | Only for Booster K1. Video encoder conversion. |
| ├── 🔶 [**`x1_twist.launch.py`**](/colcon_ws/src/hardware/Documentation/x1_twist.md) | Movement launch file. |
| │&emsp;&emsp;├── 🟢 [`twist_to_x1`](/colcon_ws/src/hardware/Documentation/twist_to_x1.md) | Robot walking. |
| │&emsp;&emsp;├── 🟢 [`pantilt_to_x1`](/colcon_ws/src/hardware/Documentation/pantilt_to_x1.md) | Head movement. |
| │&emsp;&emsp;└── 🟢 [`odom_to_tf`](/colcon_ws/src/hardware/Documentation/odom_to_tf.md) | Odometry transformation. |
| └── 🔶 [**`ball_follower.launch.py`**](/colcon_ws/src/planning/ball_follower/documentation/ball_follower_launch.md) | Ball following launch file. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;├── 🟢 [`ball_detector`](/colcon_ws/src/vision/ball_detector/documentation/ball_detector.md) | Ball detection through CV. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;├── 🟢 [`ball_follower`](/colcon_ws/src/planning/ball_follower/documentation/ball_follower.md) | Walking towards the ball. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;└── 🟢 [`head_ball_follower`](/colcon_ws/src/planning/ball_follower/documentation/head_ball_follower.md) | Following the ball with the head. |

## 🔀 Differences between the three launch files

While they share the majority of the nodes, there's some key differences that allows them to work with the specific hardware of each robot:

### 🏃‍♀️ Movement nodes

For high-level movement control, there's specific nodes for each robot model.

| Robot Model | Movement Nodes |
|---|---|
| Unitree G1 | `twist_to_g1`, `pantilt_to_g1`, `odom_to_tf` |
| Booster T1 | [`twist_to_t1`](/colcon_ws/src/hardware/booster/twist_to_t1/twist_to_t1/twist_to_t1.py), [`pantilt_to_t1`](/colcon_ws/src/hardware/booster/twist_to_t1/twist_to_t1/pantilt_to_t1.py), [`odom_to_tf`](/colcon_ws/src/hardware/booster/twist_to_t1/twist_to_t1/odom_to_tf.py) |
| Booster K1 | [`twist_to_k1`](/colcon_ws/src/hardware/booster_k1/twist_to_k1/twist_to_k1/twist_to_k1.py), [`pantilt_to_k1`](/colcon_ws/src/hardware/booster_k1/twist_to_k1/twist_to_k1/pantilt_to_k1.py), [`odom_to_tf`](/colcon_ws/src/hardware/booster_k1/twist_to_k1/twist_to_k1/odom_to_tf.py) |

> The source files for the Unitree G1 nodes aren't in this repository, but in the `/home/unitree` directory on the robot itself.

### 📹 Video encoding conversion

The native topics publishing the camera feed on the Booster K1 are encoded in `nv12`, which is not compatible with our image processing nodes. To solve this, the [`nv12_converter_node`](/colcon_ws/src/vision/boosterk1_image_proc/README.md) is included in the `K1_state_machine.launch.py` to publish video in `bgr8`.

| Robot Model | Node |
|---|---|
| Booster K1 | [`nv12_converter_node`](/colcon_ws/src/vision/boosterk1_image_proc/README.md) |

## ⚡ Running at startup

This launch file is meant to be ran automatically when the robot is powered on. To achieve this, it must be added as a systemd service with the `robot_upstart` package.

See [robot_upstart](/Documentation/Guides/robot_upstart.md) for details.

