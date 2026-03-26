from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    marker_detector_node = Node(
        package='social_vision_system',
        executable='marker_detector',
        name='marker_detector_node',
        output='screen'
    )
    return LaunchDescription([
        marker_detector_node
    ])