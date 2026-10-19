"""Run the T1 ppowalk (velocity) policy node and its web GUI.

Example:
  ros2 launch t1_ppowalk t1_ppowalk.launch.py   # pick a model in the GUI's dropdown
  ros2 launch t1_ppowalk t1_ppowalk.launch.py models_dir:=/abs/folder policy_path:=/abs/model.onnx
  # GUI: http://<robot-ip>:8084
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('t1_ppowalk')
    gui_share = get_package_share_directory('t1_ppowalk_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'models_dir', default_value='~/t1_ppowalk_models',
            description='Folder whose *.onnx models the GUI dropdown offers'),
        DeclareLaunchArgument(
            'policy_path', default_value='',
            description='Optional model loaded at startup (empty: choose one in the GUI)'),
        DeclareLaunchArgument(
            'policy_params',
            default_value=os.path.join(share, 'config', 't1_ppowalk_policy.yaml')),
        DeclareLaunchArgument(
            'enable_lowcmd', default_value='false',
            description='Publish LowCmd (robot must be in Custom mode)'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the web GUI'),

        Node(
            package='t1_ppowalk',
            executable='t1_ppowalk_policy_node',
            name='t1_ppowalk_policy_node',
            output='screen',
            parameters=[
                LaunchConfiguration('policy_params'),
                {'policy_path': LaunchConfiguration('policy_path'),
                 'models_dir': LaunchConfiguration('models_dir'),
                 'enable_lowcmd': ParameterValue(
                     LaunchConfiguration('enable_lowcmd'), value_type=bool)},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gui_share, 'launch', 't1_ppowalk_gui.launch.py')),
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
