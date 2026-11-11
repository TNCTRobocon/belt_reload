from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("belt_reload"))
    parameter_file = package_share / "config" / "belt_reload.yaml"

    return LaunchDescription([
        Node(
            package="belt_reload",
            executable="belt_reload_node",
            name="belt_reload_node",
            output="screen",
            parameters=[str(parameter_file)],
        ),
    ])
