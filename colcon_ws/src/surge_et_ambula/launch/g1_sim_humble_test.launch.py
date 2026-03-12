"""
Launch de prueba con soccer_field_test.world.
Mundo mínimo para ir agregando modelos uno por uno y depurar.

Uso:
  ros2 launch surge_et_ambula g1_sim_humble_test.launch.py

Para probar el spawn por separado (ver errores del nodo create):
  ros2 launch surge_et_ambula g1_sim_humble_test.launch.py spawn_robot:=false

Luego en otra terminal:
  source install/setup.bash
  ros2 run ros_gz_sim create -world default -topic /robot_description -name G1 -x 0 -y 0 -z 1.5
"""
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import xacro


def generate_launch_description():
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    g1_description_share = get_package_share_directory('g1_description')
    g1_prefix = get_package_prefix('g1_description')
    gazebo_envs_share = get_package_share_directory('gazebo_envs')

    gz_sim_launch = PathJoinSubstitution(
        [ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py']
    )

    urdf_file = os.path.join(
        g1_description_share,
        'urdf',
        'g1_23dof_mode_10.urdf'
    )

    robot_description_content = xacro.process_file(urdf_file).toxml()

    # Mundo de prueba: soccer_field_test.world (ir descomentando modelos en el .world)
    world_file = os.path.join(
        gazebo_envs_share,
        'worlds',
        'soccer_field_test.world'
    )

    g1_share_root = os.path.join(g1_prefix, 'share')
    gazebo_models = os.path.join(gazebo_envs_share, 'models')

    existing_gz = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    existing_ign = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    gz_paths = [p for p in [existing_gz, g1_share_root, gazebo_models] if p]
    ign_paths = [p for p in [existing_ign, g1_share_root, gazebo_models] if p]

    gz_resource_path = os.pathsep.join(gz_paths)
    ign_resource_path = os.pathsep.join(ign_paths)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    spawn_robot_arg = LaunchConfiguration('spawn_robot', default='true')
    spawn_condition = IfCondition(PythonExpression(["'", spawn_robot_arg, "' == 'true'"]))

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-topic', '/robot_description',
            '-name', 'G1',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        condition=spawn_condition
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'spawn_robot',
            default_value='true',
            description='Si false, no spawnea el robot (para ejecutar create por separado y ver errores)'
        ),

        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=gz_resource_path
        ),

        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=ign_resource_path
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': f'-r {world_file}',
                'on_exit_shutdown': 'false',
                'paused': 'false'
            }.items(),
        ),

        robot_state_publisher_node,

        TimerAction(
            period=10.0,
            actions=[spawn_robot]
        ),
    ])
