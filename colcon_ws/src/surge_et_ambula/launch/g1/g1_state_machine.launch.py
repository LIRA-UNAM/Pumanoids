from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('ball_detector'),
        'models',
        'yolov8_center.pt' # <-- Change this to the model you want to use
    )
    start_position = LaunchConfiguration('start_position')

    return LaunchDescription([
        DeclareLaunchArgument('start_position',default_value='center'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('surge_et_ambula'),
                'launch',
                'g1',
                'g1_hardware.launch.py'
                )
            )
        ),
        
        Node(
            package = 'game_planner',
            executable = 'game_planner',
            name = 'game_planner',
            output = 'screen'

        ),

        Node(
            package = 'twist_to_g1',
            executable = 'twist_to_g1',
            name = 'twist_to_g1',
            output = 'screen',
        ),

        Node(
            package = 'twist_to_g1',
            executable = 'odom_to_tf',
            name = 'odom_to_tf',
            output = 'screen',
        ),

        Node(
            package='ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{
                'show_debug_window': False,
                'model_path': model_path
            }],
            output='screen'
	),

        Node(
            package = 'position_start',
            executable = 'position_start',
            name = 'position_start',
            output = 'screen',
            arguments = [start_position],
        ),

        Node(
            package='ball_follower',
            executable='ball_follower',
            name='ball_follower',
            output='screen')

    ])
