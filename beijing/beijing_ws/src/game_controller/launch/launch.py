# coding: utf8

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _make_node(context, *args, **kwargs):
    """Build the node after parsing network options from launch arguments."""
    port_text = context.perform_substitution(LaunchConfiguration('port')).strip()
    try:
        port = int(port_text)
    except ValueError:
        raise RuntimeError(f"game_controller port must be an integer, got '{port_text}'")
    if not 1 <= port <= 65535:
        raise RuntimeError(f"game_controller port must be in [1, 65535], got {port}")

    enabled_text = context.perform_substitution(
        LaunchConfiguration('enable_ip_white_list')).strip().lower()
    enabled = enabled_text in ('1', 'true', 'yes', 'on')
    raw_ips = context.perform_substitution(LaunchConfiguration('ip_white_list'))
    ip_white_list = [ip.strip() for ip in raw_ips.split(',') if ip.strip()]

    return [Node(
        package='game_controller',
        executable='game_controller_node',
        name='game_controller',
        output='screen',
        parameters=[{
            'port': port,
            'enable_ip_white_list': enabled,
            'ip_white_list': ip_white_list,
        }],
    )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port', default_value='3838',
            description='UDP port used by GameController'),
        DeclareLaunchArgument(
            'enable_ip_white_list', default_value='true',
            description='Drop packets from IPs not in ip_white_list'),
        DeclareLaunchArgument(
            'ip_white_list', default_value='192.168.1.199',
            description='Comma-separated GameController source IPs'),
        OpaqueFunction(function=_make_node),
    ])
