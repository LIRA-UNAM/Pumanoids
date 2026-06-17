import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ball_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('my_robot'),
                'launch',
                'test_with_movement.launch.py'
            )
        )
    )

    goalkeeper_node = Node(
        package='goalkeeper',
        executable='goalkeeper_guard_executable',
        name='goalkeeper_guard',
        output='screen'
    )

    return LaunchDescription([
        ball_follower_launch,
        goalkeeper_node
    ])