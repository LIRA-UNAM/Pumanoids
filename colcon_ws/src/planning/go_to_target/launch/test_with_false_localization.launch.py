import os
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


# --- DESCRIPTION ---
# This launch file runs all the necessary nodes to get the pumas_map->pumas_base_link
# transformation WITHOUT LOCALIZATION and gets a target position to walk the robot to.
# You must start the test with the robot at the center of the field

# --- HOW TO USE ---
# To send a target position, run:
# ros2 topic pub /go_to_target/target geometry_msgs/msg/Pose2D "{x: 0.0, y: 0.0, theta: 0.0}" --once
#
# --- JOELIAN POINT ---
# If you want to test with Joelian point, run also:
# ros2 launch carry_ball_to_goal carry_ball_to_goal.launch.py
#
# And read the point with:
# ros2 topic echo /carry_ball_to_goal/point

def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    
    nodes = [

        # pumas_map->pumas_odom identity transform
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

        # Node that gets a target position and moves the robot to it.
        Node(
            package = 'go_to_target',
            executable = 'go_to_target',
            name = 'go_to_target'
        ),

        # To print the current robot position (pumas_map->pumas_base_link)
        Node(
            package = 'tf2_ros',
            executable = 'tf2_echo',
            name = 'pumas_tf2_echo',
            #output = 'screen',
            arguments = [
                'pumas_map',
                'pumas_base_link'
            ]
        )
    ]

    if robot_name == 'k1':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                    get_package_share_directory('new_twist_to_k1'),
                    'launch',
                    'k1_twist.launch.py'),
                )
            )
        )

    if robot_name == 't1':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                    get_package_share_directory('new_twist_to_t1'),
                    'launch',
                    't1_twist.launch.py'),
                )
            )
        )

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='t1',
            description='Modelo del robot'
        ),
        OpaqueFunction(function=launch_setup)
    ])

