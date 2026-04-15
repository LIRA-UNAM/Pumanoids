# x1_twist.launch.py

This launch file is responsible for launching the nodes for high-level walking and head movement control.

There's a launch file for each robot model:

- [`t1_twist.launch.py`](../booster/twist_to_t1/launch/t1_twist.launch.py) for the *Booster T1*.

- [`k1_twist.launch.py`](../booster_k1/twist_to_k1/launch/k1_twist.launch.py) for the *Booster K1*.

> Unitree G1 launch comming soon.

## Running the launch

> [!NOTE]
> These launch files are already included in the [master launch file](../../surge_et_ambula/documentation/X1_state_machine.md).

To run the 

## Launch file breakdown

Here you can visualize what nodes are launched and what's their role.

| ROS 2 Element | Description |
| :--- | :--- |
| 🔶 **`x1_twist.launch`** | Master launch file. |
| ├── 🟢 [`twist_to_x1`](./twist_to_x1.md) | Robot walking. |
| ├── 🟢 [`pantilt_to_x1`](./pantilt_to_x1.md) | Head movement. |
| └── 🟢 [`odom_to_tf`](./odom_to_tf.md) | Odometry transformation. |

##