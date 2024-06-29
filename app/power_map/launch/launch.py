import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("power_map"),
        "config",
        "default_param.yml",
    )
    return LaunchDescription(
        [
            Node(
                package="power_map",
                executable="power-map",
                namespace="app",
                parameters=[config],
                remappings=[("/app/power", "/device/order/power")],
            ),
        ],
    )
