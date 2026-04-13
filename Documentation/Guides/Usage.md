# 🚀 Usage Guide

This guide provides instructions on how to execute this project, alongside with documentation for the execution and develop of the individual nodes.

> [!WARNING]
> This guide assumes that you have already completed the installation process and have set up the development environment. If not, please refer to the [Installation Guide](./Installation.md) before proceeding.

## 🔎 General Overview

1. The robots must be turned off and placed at the side of the field at the start of the match.

> ℹ️ **to-do**: specify how the robots must be placed (laying down, standed up)

2. Turn the robots on and wait for startup.

3. The software will connect to Game Controller

> ℹ️ **to-do**: Complete this section.


## 🌕 The Master Launch

To run all the necessary nodes, there's a master launch file called `X1_state_machine.launch.py`. Replace `X` with the model of the robot being used (i.e., `K1_state_machine.launch.py` for the *Booster K1*).

> Available models:
> - Unitree G1
> - Booster T1
> - Booster K1

> 📝 See the [master launch breakdown](#what-does-it-launch) for details.

#### ⚙️ Manual launch

> [!NOTE]
> This project is meant to be [launched automatically](#automatic-launch).

To run this project manually, run:

```bash
ros2 launch surge_et_ambula X1_state_machine.launch.py
```

#### ✅ Automatic launch

For the launch to be ran at startup, it must be added as a systemd service with the [robot_upstart](../../colcon_ws/src/extras/robot_upstart/README.md) package.

> ✅ The automatic launch can be set up with the [installation script](./Installation.md#running-the-installation-script).


### What does it launch

Here's a breakdown of the elements the master launch runs:

| ROS 2 Element | Description |
| :--- | :--- |
| 🔶 [**`X1_state_machine.launch.py`**](../../colcon_ws/src/surge_et_ambula/documentation/X1_state_machine.md) | Master launch file. |
| ├── 🟢 [`game_planner.py`](../../colcon_ws/src/planning/game_planner/game_planner/game_planner.py) | General state machine. |
| ├── 🟢 [`position_start.cpp`](../../colcon_ws/src/planning/position_start/src/position_start.cpp) | Positioning at startup. |
| ├── 🟢 [`nv12_converter_node.cpp`](../../colcon_ws/src/vision/boosterk1_image_proc/src/nv12_converter_node.cpp) | Only for Booster K1. Video encoder conversion. |
| ├── 🔶 [**`x1_twist.launch.py`**](../../colcon_ws/src/hardware/documentation/x1_twist.md) | Movement launch file. |
| │&emsp;&emsp;├── 🟢 [`twist_to_x1.py`](./src/api/client.js) | Robot walking. |
| │&emsp;&emsp;├── 🟢 [`pantilt_to_x1.py`](./src/api/client.js) | Head movement. |
| │&emsp;&emsp;└── 🟢 [`odom_to_tf.py`](./src/api/client.js) | Odometry transformation. |
| └── 🔶 [**`ball_follower.launch.py`**](./src/components) | Ball following launch file. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;├── 🟢 [`ball_detector.py`](./src/api/client.js) | Ball detection through CV. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;├── 🟢 [`ball_follower.py`](./src/api/client.js) | Walking towards the ball. |
| &emsp;&emsp;&nbsp;&nbsp;&nbsp;└── 🟢 [`head_ball_follower.py`](./src/api/client.js) | Following the ball with the head. |

> ℹ️ For more details on the master launch, see the [master launch documentation](../../colcon_ws/src/surge_et_ambula/documentation/X1_state_machine.md).

## 📦 Other Packages

### ⏻ robot_upstart

This is an utility to manage ROS2 nodes as systemd services, allowing them to be **automatically started on boot** and easily managed with standard systemd tools.

See [robot_upstart](./robot_upstart.md) for more.
