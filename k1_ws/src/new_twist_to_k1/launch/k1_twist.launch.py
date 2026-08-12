from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='new_twist_to_k1',
            executable='twist_to_k1',
            name='twist_to_k1',),
#            output='screen'),
        Node(
            package='new_twist_to_k1',
            executable='pantilt_to_k1',
            name='pantilt_to_k1',),
#            output='screen'),
        Node(
            package='new_twist_to_k1',
            executable='odom_to_tf',
            name='odom_to_tf',),
#            output='screen')
    ])
