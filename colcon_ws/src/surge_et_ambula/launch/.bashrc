from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcl_ros',
            executable='voxel_grid_node',
            name='voxel_grid_filter',
            parameters=[{'leaf_size': 0.05}],
            remappings=[
                ('input', '/camera/depth/points'),
                ('output', '/camera/depth/points_filtered')
            ]
        )
    ])

