
# position_start

This is package contains the `position_start` node. This node walks the robots from the side of the field to their starting positions for the ***kick-off***

| Package | Node | Source file |
| ------- | ---- | ----------- |
| `position_start` | `position_start` | [`position_start.cpp`](src/position_start.cpp) |

## Usage

The robots must be placed on the right edge of the field (on the right touch line in the team's side, close to the corner mark).

### Running the node

To run the `position_start` node:

    ros2 run position_start position_start <target> <options>

Where `<target>` can be:

- `center`
- `left`
- `right`

Or any other target position specified in the [YAML](config/) file

Additionally, the following `<options>` are available:

- `-d` run with logging level set to `DEBUG`
- `-s` run in standalone mode (without the state machine)
- `-i` run with rotation to the right (clockwise) instead of left (counterclockwise)

### Node dependencies

#### Unitree G1

| Package | Node |
| ------- | ---- |
| `twist_to_g1` | `twist_to_g1` |
| `twist_to_g1` | `odom_to_tf` |

#### Booster T1

| Package | Node |
| ------- | ---- |
| `twist_to_t1` | `twist_to_t1` |
| `twist_to_t1` | `odom_to_tf` |

#### Booster K1

| Package | Node |
| ------- | ---- |
| `twist_to_k1` | `twist_to_k1` |
| `twist_to_k1` | `odom_to_tf` |


## Topics

| Topic | Role | Type | Description | Node related |
| ----- | ---- | ---- | ----------- | ------------ |
| `/position_start/enable` | Subscriber | `std_msgs/msg/Bool` | Get the signal to start moving the robot. | `game_planner` |
| `/position_start/finish` | Publisher | `std_msgs/msg/Bool` | Signals the machine state that the robot has finished moving. | `game_planner` |
| `/cmd_vel` | Publisher | `geometry_msgs/msg/Twist` | Sends the movement instructions to the `twist_to_t1` / `twist_to_g1` node. | `twist_to_t1` / `twist_to_g1` / `twist_to_k1` |

### Coordinate transformations

| Source frame | Target frame | Description | Node related |
| :----------: | :----------: | ----------- | ------------ |
| `odom` | `base_link` | Transformation by the `odom_to_tf` node. Used for rotation feedback. | `odom_to_tf` |

## Positions

The target positions can be found in the [config](config/) directory

They are defined by a YAML file.

### YAML structure

In order for the node to parse the YAML file, it must have the following structure:

    position_start:
      ros__parameters:
        <target1>:
          x: <float1>
          y: <float2>
        <target2>:
          x: <float3>
          y: <float4>
        <target3>:
          x: <float5>
          y: <float6>

You can add as many targets as you need.

> ⚠️ You **have to re-compile** the package after modifying the YAML file.

### YAML file selection

The node uses a `loadConfiguration` function to determine which YAML file to use

    loadConfiguration("config/positions.yaml");

## How it works

### General overview

The node gets the `x` and `y` values from the specified target, then it waits for the state machine to sent a `true` through the `/position_start/enable` topic to continue.

If the `true` is received, the robot walks forward for an `x` amount of seconds. Then it stops and rotates 90° to the left (counterclockwise). After that, it stops rotating and walks forward for `y` seconds.

### Detailed explanation

#### Initialization

The initial state of the node is `WAITING_FOR_STATE_MACHINE`. 

The first thing the node does is to look for the YAML file.

    loadConfiguration("config/positions.yaml");

If it succeeds, it'll start parsing the file, looking for the specified target. When found, it stores the `x` value of the target in the `target_x_` variable and the `y` value in the `target_y_`.

It waits for the machine state to send the authorization to start moving the robot (a boolean `true`) through the `/position_start/enable` topic.

Once it gets the authorization, the node state changes to `INITIAL_POSE`.

#### Waiting for transformation

It takes a few seconds (~3 s) to receive the transformation data. When the first data is received, the `has_initial_theta_` variable changes to `true`.

To keep track of time, the movement management code is inside a `timer_callback` function, with a period of 500 milliseconds (stored in the `timer_period` variable).

If state is `INITIAL_POSE` and it started receiving data from the transformation, the state changes to `MOVING_X`

#### First Walk

For each timer callback, the robot compares the current X timer (`current_timer_pos_x`, which initially is `0`) to the desired duration (`target_x_`). If the former is less than the latter, the robot walks forward and the X timer is incremented by `500` (the period of timer callbacks)

    current_timer_pos_x += timer_period / 1000.0;

This way, the `timer_period` variable can be modified to change the `timer_callback` period without altering the walking duration.

Once the X timer reaches the desired duration, the node state changes to `ROTATING`

#### Rotation

Checks the angular distance between the current angle of rotation and the target angle.

    angular_error = shortestAngularDistance(current_angle, target_angle);

The rotation speed is positive (counterclockwise) or negative (clockwise) depending on the sign of angular_error.

If the distance is less than 0.1 radians (~5.7°), the robot stops and state changes to `MOVING_Y`

#### Second walk

It uses the same logic as the first walk, but uses variables for the Y axis.

Once the desired walk duration is reached, the robot stops and the node state changes to `FINISHED`

#### Finished

Once the robot has finished moving, it sends a `true` to the state machine through the `/position_start/finish` topic.

## Rules to be considered

According to the [2025 Competition Rules](https://humanoid.robocup.org/wp-content/uploads/RC-HL-2025-Rules.pdf) (Pg. 47), robots can be placed anywhere on the **touch lines** or **goal lines** on the respective team's side, facing the field.

After that, they must move to their side of the field autonomously. They can't be inside the goal. If the team isn't taking the kick-off, the robots can't be inside the center circle before the ball is in play.

