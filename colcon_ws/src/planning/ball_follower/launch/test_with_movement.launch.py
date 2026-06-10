# -------------------------------
# BOOSTER ball follower test with movement nodes.
# -------------------------------
# This launch was made to quickly test the ball_detector
# and ball_follower nodes, alongside with the new_twist_to_k1 or new_twist_to_t1 nodes
# for the robot to move.
#
# To enable head movement, run:
# ros2 topic pub /head_ball_follower/enable std_msgs/msg/Bool "{data: true}"  --once
# 
# And to enable walk movement:
# ros2 topic pub /ball_follower/enable std_msgs/msg/Bool "{data: true}"  --once

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'ball_follower.launch.py'),
            ),
            launch_arguments={'robot': robot_name}.items()
        )
    ]

    if robot_name == 'k1':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                    get_package_share_directory('new_twist_to_k1'),
                    'launch',
                    'k1_twist.launch.py'),
                )
            )
        )

    if robot_name == 't1':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                    get_package_share_directory('new_twist_to_t1'),
                    'launch',
                    't1_twist.launch.py'),
                )
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='t1',
            description='Modelo del robot'
        ),
        OpaqueFunction(function=launch_setup)
    ])

