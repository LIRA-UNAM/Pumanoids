from launch.substitutions import PathJoinSubstitution, EnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
import os

def generate_launch_description():
    #1. Find darwin_description_package
    pkg_share_path = os.pathsep + os.path.join(get_package_prefix('darwin_description'), 'share')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += pkg_share_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] =  pkg_share_path

    #2. Find gazebo (gz) path 
    ros_gz_sim_pkg_path  = get_package_share_directory('ros_gz_sim')
    gz_sim_launch_path   = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    gz_spawn_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_spawn_model.launch.py'])

    #3. Find darwin_description urdf file
    description_pkg_path  = get_package_share_directory('darwin_description')
    urdf_file_path = os.path.join(description_pkg_path, 'urdf', 'darwin_lab.urdf')
    robot_description_content = xacro.process_file(urdf_file_path).toxml()
    
    #4. Find the gazebo world models (soccer field, goal, ball, etc.)
    gazebo_envs_pkg = get_package_share_directory('gazebo_envs')
    world_file = os.path.join(gazebo_envs_pkg, 'worlds', 'soccer_field.world')

    #5. Set the environment variable GZ_SIM_RESOURCE_PATH
    gz_sim_resource_path = os.pathsep + os.path.join(gazebo_envs_pkg, 'models')
    os.environ['GZ_SIM_RESOURCE_PATH'] += gz_sim_resource_path

    #4. Find robot_description params
    robot_state_publisher_params = [{'robot_description': robot_description_content}]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch_path),
            launch_arguments={
                'gz_args':[f'-r ', world_file],
                'on_exit_shutdown': 'true',
                'paused': 'true'
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_spawn_launch_path),
            launch_arguments={
                'world':'default',
                'topic':'/robot_description',
                'entity_name': 'darwin',
                'x': '-4.0',
                'y': '0.0',
                'z': '1.0',
                'Y': '1.5708'
            }.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=robot_state_publisher_params
        ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=[
        #         '--ros-args', '-p',
        #         f'config_file:={gz_bridge_params_path}'
        #     ],
        #     output='screen'
        # ),
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_file],
        # )
    ])