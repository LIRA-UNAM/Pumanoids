import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo de detección usando DeepFace (Ojos) SOLAMENTE
        Node(
            package='person_fallower',
            executable='deepface_follower_node',
            name='deepface_follower_node',
            output='screen',
            parameters=[{
                'show_debug_window': True,
                'target_face_height_px': 180.0,
                'image_topic': '/camera/color/image_raw'
            }]
        )
    ])