from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    depth = Node(
        package="pi_i2c",
        executable="depth",
        namespace="device",
    )
    return LaunchDescription([depth])
