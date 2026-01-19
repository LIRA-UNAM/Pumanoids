
# position_start package

This is package contains the `position_start` node. This node walks the robots from the side of the field to their starting positions for the ***kick-off***

## Usage

The robots must be placed on the right edge of the field (on the right touch line in the team's side, close to the corner mark).

### Booster T1

The `twist_to_t1`  must be running.

    ros2 run twist_to_t1 twist_to_t1

and `odom_to_tf` too

    ros2 run twist_to_t1 odom_to_tf

### Unitree G1

The `twist_to_G1`  must be running.

    ros2 run twist_to_g1 twist_to_g1

and `odom_to_tf` too

    ros2 run twist_to_g1 odom_to_tf

### Running the node

To run the `position_start` node:

    ros2 run position_start position_start <target>

Where `<target>` can be:

- `center`
- `left`
- `right`

Or any another target position specified in the [YAML](config/positions.yaml) file

## Topics

| Topic | Role | Type | Description |
| ----- | ---- | ---- | ----------- |
| `/position_start/enable` | Subscriber | `std_msgs/msg/Bool` | Get the signal to start moving the robot. |
| `/position_start/finish` | Publisher | `std_msgs/msg/Bool` | Signals the machine state that the robot has finished moving. |
| `/cmd_vel` | Publisher | `geometry_msgs/msg/Twist` | Sends the movement instructions to the `twist_to_t1` node |
| `/odometer_state` | Subscriber | `booster_interface/msg/Odometer` | Receives feedback from the T1 odometer (only using rotation feedback) |

## Positions

The target positions can be found in the [config](config/) directory

They are defined in a YAML file

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

> ✅ You **need** to re-compile the package after modifying the YAML file.

### YAML file selection

The node use a `loadConfiguration` function to determine which YAML file use

    loadConfiguration("config/positions.yaml");

## How it works

### General overview

The node gets the `x` and `y` values from the specified target, then it waits for the state machine to sent a `true` through the `/position_start/enable` topic.

If the `true` is received, the robot walks forward for an `x` amount of seconds. Then it stops and rotates 90° to the left (counterclockwise). After that, it stops rotating and walks forward for `y` seconds.

### Detailed explanation

#### Initialization

The state of the node at the start is `WAITING_FOR_STATE_MACHINE`. 

The first thing the node does is to look for the YAML file.

    loadConfiguration("config/positions.yaml");

If it succeeds, it'll start parsing the file, looking for the specified target. When found, it stores the `x` value of the target in the `target_x_` variable and the `y` value in the `target_y_`.

It waits for the machine state to send the authorization to start moving the robot (a boolean `true`) through the `/position_start/enable` topic.

#### Initial theta

To make the robot always rotate to the same direction, even if it doesn't walk straight, it stores the current rotation angle before it starts moving.

When the authorization is received, the state changes to `INITIAL_POSE`. In this state, the node will save the current rotation value from the odometer in the `initial_theta_` variable. After that, the state will change to `MOVING_X`.

#### First walk

To keep track of time, the movement management code is inside a `timer_callback` function, with a period of 500 milliseconds (stored in the `timer_period` variable).

[to-do]


## Rules to be considered

According to the [2025 Competition Rules](https://humanoid.robocup.org/wp-content/uploads/RC-HL-2025-Rules.pdf) (Pg. 47), robots can be placed anywhere on the **touch lines** or **goal lines** on the respective team's side, facing the field. 

After that, they must move to their side of the field autonomously. They can't be inside the goal. If the team isn't taking the kick-off, the robots can't be inside the center circle before the ball is in play.

