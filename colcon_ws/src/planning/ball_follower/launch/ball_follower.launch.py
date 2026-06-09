# ---------------------------
# This is the main launch file to use the ball_follower nodes.
# ---------------------------
# Other launch files that use or want to use the ball_follower nodes,
# like the master launch, summon this.

# NOTE: For testing, there are other launch files inside this package.

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
import yaml

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        'yolov8_center_sys_low.engine' # <-- Change this to the model you want to use
    )
    yaml_path = str( PathJoinSubstitution([
        get_package_share_directory('config_files'),
        'robot',
        PythonExpression([
            LaunchConfiguration('robot'),
            " + '.yaml'"
        ])
    ]))
    with open(yaml_path, "r") as file:
        params = yaml.safe_load(file)
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='t1',
            description = 'Modelo del robot'
        ),
        # Vision node to detect field elements
        Node(
            package='new_ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{
                'hfov': params['camera']['hfov_deg'],
                'vfov': params['camera']['vfov_deg'],
                'head_z': params['head']['height_m'],
                'ball_radius': 0.07, # NOTE: RoboCup ball is 0.11 !!!!
                'show_debug': True, # <-- Only set to True if testing through ethernet
                'model_path': model_path
            }],
            output='screen'),
        # To follow the ball with the head
        Node(
            package='ball_follower',
            executable='head_ball_follower',
            name='head_ball_follower',
            output='screen'),
        # To follow the ball walking
        Node(
            package='ball_follower',
            executable='ball_follower',
            name='ball_follower',
            output='screen')
    ])
