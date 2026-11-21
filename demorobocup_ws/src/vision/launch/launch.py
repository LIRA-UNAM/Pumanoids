import os
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def handle_configuration(context, *args, **kwargs):
    default_config_path = PathJoinSubstitution([FindPackageShare('vision'), 'config']).perform(context)
    sim = _as_bool(LaunchConfiguration('sim').perform(context))
    robot_name = LaunchConfiguration('robot_name').perform(context).strip('/') or 'blue_1'
    robot_type = (LaunchConfiguration('robot_type').perform(context).strip().lower() or 't1')

    user_cfg_dir = LaunchConfiguration('vision_config_path').perform(context)
    config_path = default_config_path  
    if user_cfg_dir and user_cfg_dir.strip():
        cand = user_cfg_dir.rstrip('/')
        if os.path.exists(os.path.join(cand, 'vision.yaml')):
            config_path = cand
        else:
            print(f"[vision launch] warning: {cand}/vision.yaml not found, fallback to {default_config_path}")
    config_file = os.path.join(config_path, 'vision.yaml')
    config_local_file = os.path.join(config_path, 'vision_local.yaml')

    show_det = LaunchConfiguration('show_det')
    show_seg = LaunchConfiguration('show_seg')
    save_data = LaunchConfiguration('save_data')
    save_depth = LaunchConfiguration('save_depth')
    offline_mode = LaunchConfiguration('offline_mode')
    save_fps = LaunchConfiguration('save_fps')
    detection_model_path = LaunchConfiguration('detection_model_path').perform(context).strip()
    segmentation_model_path = LaunchConfiguration('segmentation_model_path').perform(context).strip()
    camera_type = LaunchConfiguration('camera_type').perform(context).strip()
    camera_info_topic = LaunchConfiguration('camera_info_topic').perform(context).strip()
    use_sim_time_arg = LaunchConfiguration('use_sim_time').perform(context).strip()

    params = {
        'offline_mode': offline_mode,
        'show_det': show_det,
        'show_seg': show_seg,
        'save_data': save_data,
        'save_depth': save_depth,
        'save_fps': save_fps,
        'detection_model_path': detection_model_path,
        'segmentation_model_path': segmentation_model_path,
        'camera_type': camera_type,
    }
    remappings = []

    if sim:
        if not (user_cfg_dir and user_cfg_dir.strip()):
            try:
                bridge_share = get_package_share_directory('pumasim_booster_bridge')
                sim_vision_yaml = os.path.join(bridge_share, 'config', f'vision_{robot_type}.yaml')
                if os.path.isfile(sim_vision_yaml):
                    config_local_file = sim_vision_yaml
            except PackageNotFoundError:
                pass
        if not detection_model_path:
            params['detection_model_path'] = 'model/sim_data_det_0126.onnx'
        if not segmentation_model_path:
            params['segmentation_model_path'] = 'model/sim_data_seg_0126.onnx'
        if not camera_type:
            params['camera_type'] = 'realsense'
        if not camera_info_topic:
            camera_info_topic = f'/{robot_name}/camera/camera_info'
        params['camera_info_topic'] = camera_info_topic
        params['use_sim_time'] = _as_bool(use_sim_time_arg) if use_sim_time_arg else True
        remappings.append(('/boostercamera/head/rgb/camera_info', camera_info_topic))
    else:
        if camera_info_topic:
            params['camera_info_topic'] = camera_info_topic
        if use_sim_time_arg:
            params['use_sim_time'] = _as_bool(use_sim_time_arg)

    return [
        Node(
            package='vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            arguments=[config_file, config_local_file],
            parameters=[params],
            remappings=remappings,
        ),
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value='',
            description='Optional directory containing vision.yaml & vision_local.yaml (empty => use package default)'
        ),
        DeclareLaunchArgument(
            "offline_mode",
            default_value='false',
            description="enable offline model"
        ),
        DeclareLaunchArgument(
            "show_det",
            default_value='false',
            description="Show detection result"
        ),
        DeclareLaunchArgument(
            "show_seg",
            default_value='false',
            description="Show segmentation result"
        ),
        DeclareLaunchArgument(
            "save_data",
            default_value='true',
            description="Save recevied image data"
        ),
        DeclareLaunchArgument(
            "save_depth",
            default_value='true',
            description="Save recevied depth img data"
        ),
        DeclareLaunchArgument(
            "save_fps",
            default_value='2',
            description="Save n frames of data each second"
        ),
        DeclareLaunchArgument(
            'detection_model_path',
            default_value="",
            description="param to override detection_model.model_path. will not override if empty"
        ),
        DeclareLaunchArgument(
            'segmentation_model_path',
            default_value="",
            description="param to override segmentation_model.model_path. will not override if empty"
        ),
        DeclareLaunchArgument(
            'camera_type',
            default_value="",
            description="param to override camera.type. will not override if empty"
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Enable simulation ONNX models and camera_info wiring'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='blue_1',
            description='Simulator robot name used when sim:=true'
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='t1',
            description='Simulator robot type (t1 or k1) used when sim:=true'
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='',
            description='Optional camera_info topic override'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='',
            description='Optional use_sim_time override'
        ),
        OpaqueFunction(function=handle_configuration)
    ])