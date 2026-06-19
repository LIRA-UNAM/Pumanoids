import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    
    if robot_name == 'NONE':
        print("\n" + "="*50)
        print("[ERROR] Robot model not specified!")
        print("Usage: ros2 launch surge_et_ambula state_machine.launch.py robot:=<model_name>")
        print("Example: ros2 launch surge_et_ambula state_machine.launch.py robot:=k1")
        print("="*50 + "\n")
        sys.exit(1)


    yaml_path = os.path.join(
        get_package_share_directory('config_files'),
        'robots',
        f'{robot_name}.yaml'
    )

    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        f'yolov8_center_sys_low_{robot_name}.engine'
    )

    state_machine_config_path = '/home/booster/Pumanoids/colcon_ws/src/surge_et_ambula/configs/state_machine_parameters.yaml'
    with open(state_machine_config_path, 'r') as file:
        configs = yaml.safe_load(file)

    config_share = get_package_share_directory('config_files')
    map_yaml = os.path.join(config_share, 'maps', 'cancha_tmr.yaml')

    with open(yaml_path, "r") as file:
        params = yaml.safe_load(file)
    
    nodes = [

        # Include the ball_follower launch.
        # It runs ball_detector and the two ball_follower nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('ball_follower'),
                'launch',
                'ball_follower.launch.py'),
            ),
            launch_arguments={'robot': robot_name}.items()
        ),
        Node(
            package='carry_ball_to_goal',
            executable='goal_robot_pose_service',
            name='goal_robot_pose_service',
            parameters=[{
                'map_yaml': map_yaml,
                'attack_goal': 'negative_y',
                'distance_from_ball_m': 0.5, # 1.25 for testing. 0.5 for real game
                'ball_pose_topic': '/vision/map_ball',
            }],
#            output='screen',
        ),
        Node(
            package = 'go_to_target',
            executable = 'go_to_target',
            name = 'go_to_target',
            parameters=[{
                'v_max': params['movement']['linear_speed'],
                'w_max': params['movement']['angular_speed']
            }]
            #output = 'screen'
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

                    "start_position": [float(i) for i in configs['start_position']],
                    "player_number":configs['player_number'], 
                    "team_number":configs['team_number'], 
                    "goalkeeper":configs['goalkeeper'], 
                    "kickoff":configs['kickoff'], 
                    "kick":configs['kick']
                }
            ]
        ),
        Node(
            package = 'particle_filter',
            executable = 'map_node',
            name = 'map_node',
            output = 'screen',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pumas_map_to_pumas_odom',
            arguments=[
                '3', '3', '0',    # x y z
                '3.14', '0', '0',    # yaw pitch roll
                'pumas_map',
                'pumas_odom'
            ]
        )
    ]

    # ROBOT-DEPENDENT NODES
    if robot_name == 'k1':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                    get_package_share_directory('new_twist_to_k1'),
                    'launch',
                    'k1_twist.launch.py'),
                )
            )
        )

    if robot_name == 't1':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                    get_package_share_directory('new_twist_to_t1'),
                    'launch',
                    't1_twist.launch.py'),
                )
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='NONE',
            description='Modelo del robot'
        ),
        OpaqueFunction(function=launch_setup)
    ])

