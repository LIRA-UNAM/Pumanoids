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
                get_package_share_directory('twist_to_t1'),
                'launch',
                't1_twist_launch.py'),
            )
        ),
        Node(
            package='ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{'show_debug_window': True}],
            output='screen'),
        Node(
            package='ball_follower',
            executable='head_ball_follower',
            name='head_ball_follower',
            output='screen'),
        Node(
            package='ball_follower',
            executable='ball_follower',
            name='ball_follower',
            output='screen'),
    ])