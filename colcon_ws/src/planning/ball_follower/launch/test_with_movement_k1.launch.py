# -------------------------------
# BOOSTER K1 ball follower test with movement nodes.
# -------------------------------
# This launch was made to quickly test the ball_detector
# and ball_follower nodes, alongside with the new_twist_to_k1 nodes
# for the robot to move.
#
# To enable head movement, run:
# ros2 topic pub /head_ball_follower/enable std_msgs/msg/Bool "{data: true}"  --once
# 
# And to enable walk movement:
# ros2 topic pub /ball_follower/enable std_msgs/msg/Bool "{data: true}"  --once

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # All twist nodes for movement and odometry
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('new_twist_to_k1'),
                'launch',
                'k1_twist.launch.py'),
            )
        ),
        # Service node for the joint states values
        Node(
            package = 'joint_states_package',
            executable = 'joints_service',
            name = 'joints_service',
            output = 'screen'
        ),
        # Image encoding conversion (nv12 -> bgr8)
        Node(
            package='boosterk1_image_proc',
            executable='nv12_converter_node',
            name='nv12_converter_node',
            output='screen'
        ),
        # Vision and ball_follower nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'ball_follower.launch.py'),
            )
        )
    ])
