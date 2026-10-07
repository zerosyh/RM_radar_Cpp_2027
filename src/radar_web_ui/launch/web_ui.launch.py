"""Launch radar_web_ui with default debugging settings."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_web_ui',
            executable='web_ui_node',
            name='radar_web_ui',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8766,
                'config_path': 'configs/main_config.yaml',
            }],
        ),
    ])
