# 🎥 nv12_converter_node

This package contains the `nv12_converter_node`, which is responsible for converting the video feed from the Booster K1's camera from `nv12` encoding to `bgr8`, making it compatible with our image processing nodes.

| Package | Node | Source file |
| ------- | ---- | ----------- |
| `boosterk1_image_proc` | `nv12_converter_node` | [`nv12_converter_node.cpp`](src/nv12_converter_node.cpp) |

## Topics

| Topic | Role | Type | Description | Node related |
| ----- | ---- | ---- | ----------- | ------------ |
| `/booster_camera_bridge/StereoNetNode/rectified_image` | Subscriber | `sensor_msgs/msg/Image` | Raw video feed from the camera in `nv12` encoding. | N/A |
| `/camera/color/image_raw` | Publisher | `sensor_msgs/msg/Image` | Video feed converted to `bgr8` encoding. | [`ball_detector`](../ball_detector/ball_detector/ball_detector.py), [`goal_detector`](../ball_detector/ball_detector/goal_detector.py), [`goal_robot_pose`](../../planning/carry_ball_to_goal/carry_ball_to_goal/goal_robot_pose.py), [``]() |

## 💻 Usage

This node is meant to be executed by the [master launch file](../../../surge_et_ambula/documentation/X1_state_machine.md) for the Booster K1 at startup.

To run the `nv12_converter_node` manually:

```bash
ros2 run boosterk1_image_proc nv12_converter_node
```

## 📝 Dependencies

### 🔀 Video encoding conversion

This node requires [jetson-utils](https://github.com/dusty-nv/jetson-utils) to convert the video feed encoding efficiently using the Jetson Orin GPU.

#### 🛠️ Installation

First make sure you have the necessary dependencies with the [`jetson-inference/CMakePreBuild.sh`](https://github.com/dusty-nv/jetson-inference/blob/master/CMakePreBuild.sh)

Then, to install `jetson-utils`, run the following commands:

```bash
cd ~
git clone https://github.com/dusty-nv/jetson-utils
cd jetson-utils
mkdir build
cd build
cmake ../
make -j$(nproc)
sudo make install
sudo ldconfig
```

