from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # ── Argumentos configurables desde CLI ───────────────────────────
    visualize_arg = DeclareLaunchArgument(
        'visualize', default_value='false',
        description='Mostrar ventana OpenCV en yolo_detector'
    )
    model_arg = DeclareLaunchArgument(
        'checkpoint',
        default_value='/home/booster/Pumanoids/colcon_ws/src/vision/ball_detector/models/yolov8_center.pt',
        description='Ruta al modelo YOLO'
    )

    # ── Nodos ─────────────────────────────────────────────────────────

    yolo_detector = Node(
        package='particle_filter',
        executable='detector_node',
        name='detector_node',
        parameters=[{
            'checkpoint': LaunchConfiguration('checkpoint'),
            'visualize':  LaunchConfiguration('visualize'),
        }],
        output='screen',
    )

    particle_filter = Node(
        package='particle_filter',
        executable='particle_filter',
        name='mcl_node',
        output='screen',
    )

    soccer_map = Node(
        package='particle_filter',
        executable='map_node',
        name='map_node',
        output='screen',
    )

    return LaunchDescription([
        visualize_arg,
        model_arg,

        soccer_map,
        yolo_detector,
        particle_filter,
    ])
