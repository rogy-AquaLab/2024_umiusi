from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    led = Node(
        package="pi_led",
        excutable="pi_led",
        namespace="device",
        parameters=[],
    )
    return LaunchDescription([led])
