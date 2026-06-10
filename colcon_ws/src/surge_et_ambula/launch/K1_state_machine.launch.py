import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
def generate_launch_description():

    state_machine_config_path = '/home/booster/Pumanoids/colcon_ws/src/surge_et_ambula/configs/state_machine_parameters.yaml'
    with open(state_machine_config_path, 'r') as file:
        configs = yaml.safe_load(file)

    start_position = LaunchConfiguration('start_position')

    return LaunchDescription([

        DeclareLaunchArgument('start_position',default_value='center'),

        # Include the twist control launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('new_twist_to_k1'),
                'launch',
                'k1_twist.launch.py'),
            )
        ),

        # Include the ball_follower launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'ball_follower.launch.py'),
            )
        ),
        Node(
            package='boosterk1_image_proc',
            executable='nv12_converter_node',
            name='nv12_converter_node',
            output='screen'
        ),
        Node(
            package = 'game_planner',
            executable = 'game_planner',
            name = 'game_planner',
            output = 'screen',
            #arguments=['--ros-args', '--log-level', 'game_planner:=debug'],
            parameters=
            [
                {
                    "player_number":configs['player_number'], 
                    "team_number":configs['team_number'], 
                    "goalkeeper":configs['goalkeeper'], 
                    "kickoff":configs['kickoff'], 
                    "kick":configs['kick']
                }
            ]

        ),
        Node(
            package='joint_states_package',
            executable='joints_service',
            name='joints_service',
            output='screen',
        ),

        Node(
            package = 'position_start',
            executable = 'position_start',
            name = 'position_start',
            output = 'screen',
            arguments = [start_position],
        ),
        Node(
            package='carry_ball_to_goal',
            executable='goal_robot_pose_service',
            name='goal_robot_pose_service',
            parameters=[{
                'map_yaml': os.path.join(
                    get_package_share_directory('config_files'),
                    'maps',
                    'cancha_tmr.yaml',
                ),
                'attack_goal': 'positive_y',
                'distance_from_ball_m': 0.25,
                'ball_pose_topic': '/vision/map_ball',
            }],
            output='screen',
        )
    ])
