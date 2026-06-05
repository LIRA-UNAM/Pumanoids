import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        'yolov8_center_sys_low.engine' # <-- Change this to the model you want to use
    )
    return LaunchDescription([
        Node(
            package='boosterk1_image_proc',
            executable='nv12_converter_node',
            name='nv12_converter_node',
            output='screen'
        ),
        Node(
            package='new_ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{
                'show_debug': False,                
                'model_path': model_path
            }],
            output='screen'),
        Node(
            package='ball_follower',
            executable='head_ball_follower',
            name='head_ball_follower',
            output='screen'),
        Node(
            package='ball_follower',
            executable='ball_follower',
            name='ball_follower',
            output='screen')
    ])
