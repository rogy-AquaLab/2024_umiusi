from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera_reader",
                executable="camera_reader",
                namespace="device",
                remappings=[("/device/camera_image", "/packet/camera_image")],
            ),
        ]
    )
