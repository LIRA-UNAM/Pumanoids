from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("t1_ppowalk_gui")
    return LaunchDescription([
        Node(
            package="t1_ppowalk_gui",
            executable="t1_ppowalk_gui",
            name="t1_ppowalk_gui_bridge",
            output="screen",
            parameters=[share + "/config/gui.yaml"],
        ),
    ])
