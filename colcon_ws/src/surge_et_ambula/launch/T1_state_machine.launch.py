import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    start_position = LaunchConfiguration('start_position')

    return LaunchDescription([

        DeclareLaunchArgument('start_position',default_value='center'),

        # Include the twist control launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('twist_to_t1'),
                'launch',
                't1_twist_launch.py'),
            )
        ),

        # Include the ball_follower launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'ball_follower_launch.py'),
            )
        ),

        Node(
            package = 'game_planner',
            executable = 'game_planner',
            name = 'game_planner',
            output = 'screen'
        ),

        Node(
            package = 'position_start',
            executable = 'position_start',
            name = 'position_start',
            output = 'screen',
            arguments = [start_position],
        ),
    ])