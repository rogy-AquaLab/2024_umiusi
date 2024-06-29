from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="imshow",
                executable="imshow",
                namespace="app",
                remappings=[("/app/camera/image", "/packet/camera/image")],
            ),
        ],
    )
