from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value="",
                description="Path to cf_bridge.yaml",
            ),
            Node(
                package="cf_bridge",
                executable="cf_bridge_node",
                name="cf_bridge",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
        ]
    )