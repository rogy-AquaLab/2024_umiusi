from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    receiver = Node(
        package="nucleo_communicate_py",
        executable="receiver",
        namespace="device",
        remappings=[
            ("/device/flex_1", "/packet/sensor/flex_1"),
            ("/device/flex_2", "/packet/sensor/flex_2"),
            ("/device/receiver_launch", "/packet/sensor/current"),
            ("/device/receiver_launch", "/packet/sensor/voltage"),
        ],
    )
    return LaunchDescription([receiver])
