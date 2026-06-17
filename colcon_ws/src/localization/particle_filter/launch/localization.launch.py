from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory
def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        'yolov8_center_sys_low.engine' # <-- Change this to the model you want to use
    )
    return LaunchDescription([
        
        Node(
            package = 'particle_filter',
            executable = 'bridge_odom',
            name = 'bridge_odom',
            output = 'screen',
        ),
        Node(
            package = 'particle_filter',
            executable = 'map_node',
            name = 'map_node',
            output = 'screen',
        ),
        Node(
            package = 'particle_filter',
            executable = 'mcl_node',
            name = 'mcl_node',
            output = 'screen',
        )
    ])
