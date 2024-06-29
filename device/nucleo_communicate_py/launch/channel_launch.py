from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    main = Node(
        package="nucleo_communicate_py",
        executable="channel",
        namespace="device",
    )
    return LaunchDescription([main])
