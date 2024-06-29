from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sender = Node(
        package="nucleo_communicate_py",
        executable="sender",
        namespace="device",
    )
    return LaunchDescription([sender])
