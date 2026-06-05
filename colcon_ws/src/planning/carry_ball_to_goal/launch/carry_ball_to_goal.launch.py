import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Stack carry_ball_to_goal.

    Si este launch se incluye desde K1_state_machine (u otro que ya levante
    ball_follower.launch): ball_detector, head_ball_follower y twist quedan
    comentados abajo para no duplicar nodos.

    Para prueba standalone, descomenta lo que necesites (p. ej. twist T1 y
    ball_detector).
    """
    config_share = get_package_share_directory('config_files')
    map_yaml = os.path.join(config_share, 'maps', 'cancha_tmr.yaml')

    return LaunchDescription([
        # twist: en K1 ya está k1_twist; en T1 usar t1_twist_launch
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('surge_et_ambula'),
        #             'launch',
        #             't1_twist_launch.py'),
        #     )
        # ),
        # Duplicado con ball_follower/launch/ball_follower.launch.py (K1)
        # Node(
        #     package='ball_follower',
        #     executable='head_ball_follower',
        #     name='head_ball_follower',
        #     output='screen',
        # ),
        # Duplicado con ball_follower/launch/ball_follower.launch.py (K1)
        # Node(
        #     package='ball_detector',
        #     executable='ball_detector',
        #     name='ball_detector',
        #     output='screen',
        # ),
       # Node(
       #     package='ball_detector',
       #     executable='goal_detector',
       #     name='goal_detector',
       #     parameters=[{'show_debug_window': False}],
       #     output='screen',
       # ),
        Node(
            package='carry_ball_to_goal',
            executable='goal_robot_pose_service',
            name='goal_robot_pose_service',
            parameters=[{
                'map_yaml': map_yaml,
                'attack_goal': 'negative_y',
                'distance_from_ball_m': 1.25,
                'ball_pose_topic': '/vision/map_ball',
            }],
            output='screen',
        ),
    ])
