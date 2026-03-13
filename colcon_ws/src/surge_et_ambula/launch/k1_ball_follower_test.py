from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'twist_to_k1',
            executable = 'twist_to_k1',
            name = 'twist_to_k1',
            output = 'screen',
        ),

        Node(
            package = 'twist_to_k1',
            executable = 'odom_to_tf',
            name = 'odom_to_tf',
            output = 'screen',
        )
    ])
