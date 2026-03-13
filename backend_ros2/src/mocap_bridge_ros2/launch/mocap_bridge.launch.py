from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value="",
                description="Path to mocap.yaml",
            ),
            Node(
                package="mocap_bridge_ros2",
                executable="mocap_bridge_node",
                name="mocap_bridge_node",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
        ]
    )