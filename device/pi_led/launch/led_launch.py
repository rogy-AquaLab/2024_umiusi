import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("pi_led"),
        "config",
        "default_param.yml",
    )
    led = Node(
        package="pi_led",
        excutable="pi_led",
        namespace="device",
        parameters=[config],
    )
    return LaunchDescription([led])
