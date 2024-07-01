from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    all_exec = Node(
        package="pi_i2c",
        executable="all",
        namespace="device",
    )
    return LaunchDescription([all_exec])
