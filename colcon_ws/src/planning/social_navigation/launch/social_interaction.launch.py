import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Nodo de la máquina de estados
    sm_node = Node(
        package='social_navigation',
        executable='greet_and_return_sm',
        name='greet_and_return_sm',
        output='screen'
    )

    # Incluir el launch de percepción y seguimiento físico (Ojos, Cuello y Piernas)
    person_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('person_fallower'), 'launch', 'person_fallower.launch.py')
        )
    )

    # Incluir el launch del agente de IA (agente_SD)
 #   agente_sd_launch = IncludeLaunchDescription(
 #       PythonLaunchDescriptionSource(
 #           os.path.join(get_package_share_directory('llm_planning'), 'launch', 'agente_SD.launch.py')
 #       )
 #   )

    return LaunchDescription([
        person_follower_launch,
        sm_node
        #agente_sd_launch
    ])