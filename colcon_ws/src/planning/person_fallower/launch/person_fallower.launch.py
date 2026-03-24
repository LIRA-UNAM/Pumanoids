import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Include the twist control launch (El driver de los motores del robot)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                get_package_share_directory('twist_to_t1'),
                'launch',
                't1_twist.launch.py'),
            )
        ),
        
        # Nodo principal de detección usando DeepFace (Ojos)
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
        ),
        
        # Nodo para el control de la base publicando Twist (Piernas)
        Node(
            package='person_fallower',
            executable='person_fallower',
            name='person_fallower',
            output='screen'
        )
    ])
