import os
import sys
import yaml
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context: LaunchContext, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    
    # 1. Validación de seguridad (Te obliga a especificar k1 o t1)
    if robot_name == 'NONE':
        print("\n" + "="*50)
        print("[ERROR] Modelo de robot no especificado!")
        print("Uso: ros2 launch <tu_paquete> <tu_archivo>.launch.py robot:=<k1_o_t1>")
        print("Ejemplo: ros2 launch particle_filter test_localization.launch.py robot:=k1")
        print("="*50 + "\n")
        sys.exit(1)

    # 2. Cargar los parámetros de hardware desde el archivo YAML
    yaml_path = os.path.join(
        get_package_share_directory('config_files'),
        'robots',
        f'{robot_name}.yaml'
    )
    
    with open(yaml_path, "r") as file:
        params = yaml.safe_load(file)

    # 3. Ruta dinámica del modelo YOLO según el robot
    model_path = os.path.join(
        get_package_share_directory('new_ball_detector'),
        'models',
        f'yolov8_center_sys_low_{robot_name}.engine'
    )
    
    # 4. Nodos BASE (los que siempre se ejecutan, sin importar el robot)
    nodes = [
        Node(
            package='particle_filter',
            executable='bridge_odom',
            name='bridge_odom',
            output='screen',
        ),
        Node(
            package='new_ball_detector',
            executable='ball_detector',
            name='ball_detector',
            parameters=[{
                'camera_topic': params['camera']['image_topic'],
                'hfov': params['camera']['hfov_deg'],
                'vfov_rad': params['camera']['vfov_deg'], # Adaptado al nombre de tu parámetro en C++
                'head_z': params['head']['height_m'],
                'show_debug': True,                
                'model_path': model_path
            }],
            output='screen'
        ),
        Node(
            package='particle_filter',
            executable='map_node',
            name='map_node',
            output='screen',
        ),
        Node(
            package='particle_filter',
            executable='mcl_node',
            name='mcl_node',
            output='screen',
        ),
        Node(
            package='joint_states_package',
            executable='joints_service',
            name='joints_service',
        )
    ]

    # 5. Nodos CONDICIONALES (Se agregan a la lista solo si es el K1)
    if robot_name == 'k1':
        nodes.append(
            Node(
                package='boosterk1_image_proc',
                executable='nv12_converter_node',
                name='nv12',
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='new_twist_to_k1',
                executable='odom_to_tf',
                name='odom_to_tf',
                output='screen',
            )
        )

    if robot_name == 't1':
        nodes.append(
            Node(
                package='new_twist_to_t1',
                executable='odom_to_tf',
                name='odom_to_tf',
                output='screen',
            )
        )
        nodes.append(
            Node(
                package='new_twist_to_t1',
                executable='twist_to_t1',
                name='twist_to_t1',
                output='screen',
            )
        )


    return nodes

def generate_launch_description():
    # Volvemos a poner NONE por defecto para que la validación de arriba funcione
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='NONE', 
        description='Modelo del robot (k1 o t1)'
    )

    return LaunchDescription([
        robot_arg,
        OpaqueFunction(function=launch_setup)
    ])
