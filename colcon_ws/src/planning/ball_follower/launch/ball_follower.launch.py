from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ball_detector',
            executable='ball_detector',
            name='ball_detector',
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