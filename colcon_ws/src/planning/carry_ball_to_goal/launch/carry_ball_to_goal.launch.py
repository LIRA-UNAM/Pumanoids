import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Incluir t1_twist_launch de surge_et_ambula
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('surge_et_ambula'),
                    'launch',
                    't1_twist_launch.py'),
            )
        ),
        # Nodo head_ball_follower
        Node(
            package='ball_follower',
            executable='head_ball_follower',
            name='head_ball_follower',
            output='screen',
        ),
        # Nodo ball_detector
        Node(
            package='ball_detector',
            executable='ball_detector',
            name='ball_detector',
            output='screen',
        ),
        # Nodo goal_detector
        Node(
            package='ball_detector',
            executable='goal_detector',
            name='goal_detector',
            parameters=[{'show_debug_window': True}],
            output='screen',
        ),
        # Nodo goal_robot_pose (interpola ball y goal_center, publica pose objetivo)
        Node(
            package='carry_ball_to_goal',
            executable='goal_robot_pose',
            name='goal_robot_pose',
            output='screen',
        ),
    ])
