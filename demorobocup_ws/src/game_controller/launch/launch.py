# coding: utf8

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_game_controller(context):
    sim = _as_bool(LaunchConfiguration('sim').perform(context))
    port = int(LaunchConfiguration('port').perform(context))
    wl_arg = LaunchConfiguration('enable_ip_white_list').perform(context).strip()
    enable_whitelist = _as_bool(wl_arg) if wl_arg else (not sim)

    ip_arg = LaunchConfiguration('ip_white_list').perform(context).strip()
    if ip_arg:
        ip_list = [ip.strip() for ip in ip_arg.split(',') if ip.strip()]
    else:
        ip_list = ['127.0.0.1'] if sim else ['172.169.80.24']

    return [
        Node(
            package='game_controller',
            executable='game_controller_node',
            name='game_controller',
            output='screen',
            parameters=[
                {
                    # 监听的端口，默认为 GameController 广播端口 3838
                    "port": port,
                    # 是否开启 IP 白名单，开启后会忽略非白名单 IP 发的广播消息，默认不应该开启
                    "enable_ip_white_list": enable_whitelist,
                    # 只接收指定 IP
                    "ip_white_list": ip_list,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('port', default_value='3838'),
        DeclareLaunchArgument('enable_ip_white_list', default_value=''),
        DeclareLaunchArgument('ip_white_list', default_value=''),
        OpaqueFunction(function=_launch_game_controller),
    ])
