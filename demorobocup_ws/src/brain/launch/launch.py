# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ['1', 'true', 'True', 'yes', 'on']


def handle_configuration(context, *args, **kwargs):
    sim = _as_bool(context.perform_substitution(LaunchConfiguration('sim')))
    robot_name = (context.perform_substitution(LaunchConfiguration('robot_name')).strip().strip('/') or 'blue_1')
    robot_type = (context.perform_substitution(LaunchConfiguration('robot_type')).strip().lower() or 't1')
    user_vision_dir = context.perform_substitution(LaunchConfiguration('vision_config_path')).strip()
    default_vision_dir = os.path.join(get_package_share_directory('vision'), 'config')

    vision_config_dir = user_vision_dir
    user_main_file = os.path.join(vision_config_dir, 'vision.yaml')
    reason = ""
    if not vision_config_dir:
        reason = "empty vision_config_path"
    elif not os.path.isdir(vision_config_dir):
        reason = f"directory '{vision_config_dir}' not found"
    elif not os.path.isfile(user_main_file):
        reason = f"'{user_main_file}' missing"

    if reason:
        print(f"[brain launch] {reason}, fallback to '{default_vision_dir}'")
        vision_config_dir = default_vision_dir

    vision_config_file = os.path.join(vision_config_dir, 'vision.yaml')
    vision_config_local_file = os.path.join(vision_config_dir, 'vision_local.yaml')

    bridge_share = None
    if sim:
        try:
            bridge_share = get_package_share_directory('pumasim_booster_bridge')
        except PackageNotFoundError:
            bridge_share = None
        if bridge_share and vision_config_dir == default_vision_dir and not os.path.isfile(vision_config_local_file):
            sim_vision_yaml = os.path.join(bridge_share, 'config', f'vision_{robot_type}.yaml')
            if os.path.isfile(sim_vision_yaml):
                vision_config_local_file = sim_vision_yaml

    if not os.path.isfile(vision_config_local_file):
        print(f"[brain launch] optional local override '{vision_config_local_file}' not present")

    config_path = os.path.join(os.path.dirname(__file__), '../config')
    config_file = os.path.join(config_path, 'config.yaml') 
    config_local_file = os.path.join(config_path, 'config_local.yaml') 

    behavior_trees_dir = os.path.join(os.path.dirname(__file__), '../behavior_trees')
    def make_tree_path(name):
        if not name.endswith('.xml'):
            name += '.xml'
        return os.path.join(behavior_trees_dir, name)
    tree = context.perform_substitution(LaunchConfiguration('tree'))
    tree_path = make_tree_path(tree)

    # 这里的 config 覆盖 config_file 中相同字段(如有) 用于在 launch 时快速指定参数, 而不需要频繁修改 config.yaml
    config = {
            # 加载哪个行为树文件
            "tree_file_path": tree_path,
            "vision_config_path": vision_config_file,
            "vision_config_local_path": vision_config_local_file,
    }

    # 简化字符串判空
    start_pos = context.perform_substitution(LaunchConfiguration('pos'))
    if start_pos:
        config['game.player_start_pos'] = start_pos
    role = context.perform_substitution(LaunchConfiguration('role'))
    if role:
        config['game.player_role'] = role
    team_id = context.perform_substitution(LaunchConfiguration('team_id')).strip()
    if team_id:
        config['game.team_id'] = int(team_id)
    player_id = context.perform_substitution(LaunchConfiguration('player_id')).strip()
    if player_id:
        config['game.player_id'] = int(player_id)
    num_players = context.perform_substitution(LaunchConfiguration('number_of_players')).strip()
    if num_players:
        config['game.number_of_players'] = int(num_players)
    gc_ip = context.perform_substitution(LaunchConfiguration('game_control_ip')).strip()
    if gc_ip:
        config['game_control_ip'] = gc_ip

    if sim:
        config['use_sim_time'] = True

    use_mujoco_loc = _as_bool(context.perform_substitution(LaunchConfiguration('use_mujoco_localization')))
    mujoco_loc_topic = context.perform_substitution(LaunchConfiguration('mujoco_localization_topic')).strip()
    if sim or use_mujoco_loc or mujoco_loc_topic:
        config['locator.use_mujoco_localization'] = use_mujoco_loc
        config['locator.mujoco_localization_topic'] = (
            mujoco_loc_topic or f'/{robot_name}/ground_truth/localization'
        )

    disableLog = context.perform_substitution(LaunchConfiguration('disable_log'))
    if _as_bool(disableLog):
        config['rerunLog.enable_file'] = False
        config['rerunLog.enable_tcp'] = False

    disableCom = context.perform_substitution(LaunchConfiguration('disable_com'))
    if _as_bool(disableCom):
        config['enable_com'] = False

    params = [config_file]
    if os.path.isfile(config_local_file):
        params.append(config_local_file)
    if sim and bridge_share:
        sim_brain_yaml = os.path.join(bridge_share, 'config', f'brain_{robot_type}.yaml')
        if os.path.isfile(sim_brain_yaml):
            params.append(sim_brain_yaml)
    params.append(config)

    return [
        Node(
            package ='brain',
            executable='brain_node',
            output='screen',
            parameters=params
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vision_config_path',
            default_value=os.path.join(get_package_share_directory('vision'), 'config'),
            description='Directory containing vision.yaml / vision_local.yaml'
        ),
        # 需要可以通过 ros2 launch brain launch.py param:=value 形式提供的参数, 需要在这里用 DeclarelaunchArgument 声明, 然后在 handle_configuration 处理
        DeclareLaunchArgument(
            'tree', 
            default_value='game.xml',
            description='Specify behavior tree file name. DO NOT include full path, file should be in src/brain/config/behavior_trees'
        ),
        DeclareLaunchArgument(
            'pos', 
            default_value='',
            description='如果需要覆盖 config.yaml 中的 game.player_start_pos, 可以在 launch 时指定参数 pos:=left'
        ),
        DeclareLaunchArgument(
            'role', 
            default_value='',
            description='如果需要覆盖 config.yaml 中的 game.player_role, 可以在 launch 时指定参数 role:=striker'
        ),
        DeclareLaunchArgument(
            'sim', 
            default_value='false',
            description='是否在仿真中'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='blue_1',
            description='Nombre del robot en simulacion para topicos por robot (p.ej. blue_1)'
        ),
        DeclareLaunchArgument(
            'robot_type',
            default_value='t1',
            description='仿真机型覆盖 (t1 | k1), 仅在 sim:=true 时生效'
        ),
        DeclareLaunchArgument(
            'use_mujoco_localization',
            default_value='false',
            description='true para usar la localizacion exacta de MuJoCo; false para usar SelfLocate interno del demo'
        ),
        DeclareLaunchArgument(
            'mujoco_localization_topic',
            default_value='',
            description='Topico Odometer de localizacion MuJoCo (por defecto /<robot_name>/ground_truth/localization)'
        ),
        DeclareLaunchArgument(
            'team_id',
            default_value='',
            description='可选覆盖 game.team_id'
        ),
        DeclareLaunchArgument(
            'player_id',
            default_value='',
            description='可选覆盖 game.player_id'
        ),
        DeclareLaunchArgument(
            'number_of_players',
            default_value='',
            description='可选覆盖 game.number_of_players'
        ),
        DeclareLaunchArgument(
            'game_control_ip',
            default_value='',
            description='可选覆盖 game_control_ip'
        ),
        DeclareLaunchArgument(
            'disable_log', 
            default_value='false',
            description='强制禁用在文件中记录日志'
        ),
        DeclareLaunchArgument(
            'disable_com', 
            default_value='false',
            description='强制禁用开启通信'
        ),
        OpaqueFunction(function=handle_configuration) # 转到 handle_configuration 中继续处理
    ])
