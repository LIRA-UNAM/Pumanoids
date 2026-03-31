import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Incluir el archivo launch del paquete person_fallower
    person_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('person_fallower'), 'launch', 'person_fallower.launch.py')
        )
    )

    greet_and_return_sm_node = Node(
        package='social_navigation',
        executable='greet_and_return_sm',
        name='greet_and_return_sm',
        output='screen'
    )

    return LaunchDescription([
        person_follower_launch,
        greet_and_return_sm_node
    ])