import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return_method_arg = DeclareLaunchArgument(
        'return_method',
        default_value='odometry',
        description='Método de retorno para la máquina de estados: odometry, marker, memory'
    )

    # Incluir el archivo launch del paquete person_fallower
    person_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('person_fallower'), 'launch', 'person_fallower.launch.py')
        )
    )

    # Incluir el archivo launch del paquete social_vision_system
    vision_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('social_vision_system'), 'launch', 'vision.launch.py')
        )
    )

    greet_and_return_sm_node = Node(
        package='social_navigation',
        executable='greet_and_return_sm',
        name='greet_and_return_sm',
        output='screen',
        parameters=[{
            'return_method': LaunchConfiguration('return_method')
        }]
    )

    return LaunchDescription([
        return_method_arg,
        person_follower_launch,
        vision_system_launch,
        greet_and_return_sm_node
    ])