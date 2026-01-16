from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    start_position = LaunchConfiguration('start_position')

    return LaunchDescription([
        DeclareLaunchArgument('start_position',default_value='center'),


        Node(
            package = 'game_planner',
            executable = 'game_planner',
            name = 'game_planner',
            output = 'screen'

        ),

        Node(
            package = 'twist_to_g1',
            executable = 'twist_to_g1',
            name = 'twist_to_g1',
            output = 'screen',
        ),

        Node(
            package = 'position_start',
            executable = 'position_start',
            name = 'position_start',
            output = 'screen',
            arguments = [start_position],
        ),
    ])