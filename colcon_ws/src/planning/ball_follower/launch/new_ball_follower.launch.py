import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Include the twist control launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('new_twist_to_k1'),
                'launch',
                'k1_twist.launch.py'),
            )
        ),
        # Service node to respond the joint states values (used by head_ball_follower and ball_follower)
        Node(
            package = 'joint_states_package',
            executable = 'joints_service',
            name = 'joints_service',
            output = 'screen'
        ),
        # Include the ball_follower launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'ball_follower.launch.py'),
            )
        )
    ])
