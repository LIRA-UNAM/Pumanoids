# ---------------------------
# This is the main launch file to use the ball_follower nodes.
# ---------------------------
# Other launch files that use or want to use the ball_follower nodes,
# like the master launch, must summon this file.

# NOTE: For testing with robot movement, there are other launch files inside this package.

import os
import sys
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

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
    
    with open(yaml_path, "r") as file:
        params = yaml.safe_load(file)
        
    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        f'yolov8_center_sys_low_{robot_name}.engine'
    )

    nodes = [
        Node(
            package = 'joint_states_package',
            executable = 'joints_service',
            name = 'joints_service',
           # output = 'screen'
        ),
        Node(
            package='new_ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{
                'camera_topic': params['camera']['image_topic'],
                'hfov': params['camera']['hfov_deg'],
                'vfov': params['camera']['vfov_deg'],
                'head_z': params['head']['height_m'],
                'ball_radius': 0.11, # NOTE: RoboCup ball is 0.11 !!!!
                'show_debug': True,
                'model_path': model_path
            }],
            #output='screen'
        ),
        Node(
            package='ball_follower',
            executable='head_ball_follower',
            name='head_ball_follower',
            parameters=[{
                'camera_topic': params['camera']['image_topic']
            }]
            #output='screen'
        ),
        Node(
            package='ball_follower',
            executable='ball_follower',
            name='ball_follower',
            #output='screen'
        )
    ]
    if robot_name == 'k1':
        nodes.append(
            Node(
                package='boosterk1_image_proc',
                executable='nv12_converter_node',
                name='nv12_converter_node',
                #output='screen',
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
