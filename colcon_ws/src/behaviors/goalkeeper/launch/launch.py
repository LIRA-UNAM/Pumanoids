from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ball_detector',
            executable='ball_detector',
            output=''),
        Node(
            package='ball_detector',
            executable='head_ball_follower',
            output=''),
        Node(
            package='twist_to_t1',
            executable='pantilt_to_t1',
            output='')
    ])