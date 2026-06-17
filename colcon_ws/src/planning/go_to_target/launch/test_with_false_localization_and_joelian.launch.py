import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# --- DESCRIPTION ---
# This launch runs all the necesary nodes to test the joelian point with false localization.
# carry_ball_to_goal publishes the joelian point where the robot needs to move to kick the ball
# and the user must publish the point to the /go_to_target/target topic for the robot to move to it.
#
# --- HOW TO USE ---
# To get the joelian point, run:
# ros2 topic echo /carry_ball_to_goal/point
#
# And to move the robot to that point, run:
# ros2 topic pub /go_to_target/target geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 0.0}" --once

def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    
    yaml_path = os.path.join(
        get_package_share_directory('config_files'),
        'robots',
        f'{robot_name}.yaml'
    )

    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        'yolov8_center_sys_low.engine' # <-- Change this to the model you want to use
    )

    config_share = get_package_share_directory('config_files')
    map_yaml = os.path.join(config_share, 'maps', 'cancha_tmr.yaml')

    with open(yaml_path, "r") as file:
        params = yaml.safe_load(file)
    
    nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('go_to_target'),
                'launch',
                'test_with_false_localization.launch.py'),
            ),
            launch_arguments={'robot': robot_name}.items()
        ),

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
            output='screen'
        ),
        Node(
            package='carry_ball_to_goal',
            executable='goal_robot_pose_service',
            name='goal_robot_pose_service',
            parameters=[{
                'map_yaml': map_yaml,
                'attack_goal': 'negative_y',
                'distance_from_ball_m': 1.25, # 1.25 for testing. 0.5 for real game
                'ball_pose_topic': '/vision/map_ball',
            }],
            output='screen',
        ),
    ]

    if robot_name == 'k1':
        nodes.append(
            Node(
                package='boosterk1_image_proc',
                executable='nv12_converter_node',
                name='nv12_converter_node',
                output='screen',
            ),
        ),
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
            default_value='t1',
            description='Modelo del robot'
        ),
        OpaqueFunction(function=launch_setup)
    ])

