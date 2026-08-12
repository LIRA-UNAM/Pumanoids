import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    mcl_params = os.path.join(
        get_package_share_directory('particle_filter'),
        'config',
        'mcl_node.yaml'
    )
    return LaunchDescription([

        Node(
            package='particle_filter',
            executable='vision_detections_bridge',
            name='vision_detections_bridge',
            output='screen',
        ),
        Node(
            package='particle_filter',
            executable='map_node',
            name='map_node',
            output='screen',
        ),
        Node(
            package='particle_filter',
            executable='mcl_node',
            name='mcl_node',
            parameters=[mcl_params],
            output='screen',
        ),
        Node(
            package='new_twist_to_t1',
            executable='odom_to_tf',
            name='odom_to_tf',
            output='screen',
        ),
        Node(
            package='soccer_field_map_generator',
            executable='live_map',
            name='soccer_field_live_map',
            output='screen',
        )
    ])
