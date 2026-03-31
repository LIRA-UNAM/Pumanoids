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
        ),
        
        # Nodo para el control de la base publicando Twist (Piernas)
        Node(
            package='person_fallower',
            executable='person_fallower',
            name='person_fallower',
            output='screen'
        )
    ])
