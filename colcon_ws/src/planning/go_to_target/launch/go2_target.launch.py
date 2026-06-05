import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config_share = get_package_share_directory('config_files')
    map_yaml = os.path.join(config_share, 'maps', 'cancha_tmr.yaml')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'new_ball_follower.launch.py'),
            )
        ),
         Node(
             package='tf2_ros',
             executable='static_transform_publisher',
             name='pumas_map_to_pumas_odom',
             arguments=[
                 '0', '0', '0',    # x y z
                 '0', '0', '0',    # yaw pitch roll
                 'pumas_map',
                 'pumas_odom'
             ]
         ),
         Node(
            package='carry_ball_to_goal',
            executable='goal_robot_pose_service',
            name='goal_robot_pose_service',
            parameters=[{
                'map_yaml': map_yaml,
                'attack_goal': 'positive_y',
                'distance_from_ball_m': 0.25,
                'ball_pose_topic': '/vision/map_ball',
            }],
            output='screen',
        ),
        Node(
             package = 'go_to_target',
             executable = 'go_to_target',
             name = 'go_to_target',
            ),
    ])