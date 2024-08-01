import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("camera_reader"),
        "config",
        "default_param.yml",
    )
    return LaunchDescription(
        [
            Node(
                package="camera_reader",
                executable="camera",
                namespace="device",
                parameters=[config],
                remappings=[("/device/camera_image", "/packet/camera_image")],
            ),
        ],
    )
