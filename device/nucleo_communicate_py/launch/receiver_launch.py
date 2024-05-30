from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    receiver = Node(
        package="nucleo_communicate_py",
        executable="receiver",
        namespace="device"
    )
    return LaunchDescription([receiver])
