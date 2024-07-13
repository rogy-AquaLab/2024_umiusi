from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    sender = Node(
        package="nucleo_communicate_py",
        executable="sender",
        namespace="device",
        remappings=[
            ("/device/quit", "/packet/order/quit"),
            ("/device/order/power", "/packet/order/power"),
        ],
    )
    return LaunchDescription([sender])
