import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Ojos: Detector de rostros
        Node(
            package='face_detector',
            executable='face_detector',
            name='face_detector',
            output='screen'
        ),
        # Cuello: Seguidor de cabeza
        Node(
            package='person_fallower',
            executable='deepface_follower_node',
            name='deepface_follower_node',
            output='screen'
        )
    ])