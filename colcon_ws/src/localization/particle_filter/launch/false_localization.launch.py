from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pumas_map_to_pumas_odom',
        arguments=[
            '0', '0', '0',    # x y z
            '0', '0', '0',    # yaw pitch roll
            'pumas_map',
            'pumas_odom'
        ]
    ),

    Node(
        package = 'new_twist_to_k1',
        executable = 'twist_to_k1',
        name = 'twist_to_k1',
    ),

    Node(
        package = 'new_twist_to_k1',
        executable = 'odom_to_tf',
        name = 'odom_to_tf',
    )

    ])

