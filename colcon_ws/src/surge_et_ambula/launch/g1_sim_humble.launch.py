from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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

    # Ruta absoluta al config de control (necesaria para gz_ros2_control en Gazebo)
    g1_control_config = os.path.join(g1_description_share, 'config', 'g1_control.yaml')
    robot_description_content = robot_description_content.replace(
        'CONFIG_PATH_PLACEHOLDER', g1_control_config
    )

    world_file = os.path.join(
        gazebo_envs_share,
        'worlds',
        'soccer_field.world'
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
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Bridge /clock para use_sim_time (necesario para sincronía con Gazebo)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'default',
            '-topic', '/robot_description',
            '-name', 'G1',
            '-x', '-4.0',
            '-y', '0.0',
            '-z', '0.8',
            '-Y', '1.5708'
        ]
    )

    # Spawn controladores: plugin configurado con namespace / para controller_manager en raíz
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    spawn_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['g1_trajectory_controller', '-c', '/controller_manager'],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
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
                'gz_args': world_file,  # Sin -r: sim pausada al inicio
                'on_exit_shutdown': 'false',
                'paused': 'true'
            }.items(),
        ),

        robot_state_publisher_node,
        clock_bridge,

        TimerAction(
            period=10.0,
            actions=[spawn_robot]
        ),
        # Spawn controladores ~5s después del robot
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='[G1 Sim] Cargando controladores... Espera "Successfully loaded" y luego pulsa Play en Gazebo.'),
                spawn_joint_state_broadcaster,
                spawn_trajectory_controller,
            ]
        ),
    ])