from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    receiver = Node(
        package="nucleo_communicate_py",
        executable="receiver",
        namespace="device",
        remappings=[
            ("/device/receiver_launch", "/packet/sensor/flex/1"),
            ("/device/receiver_launch", "/packet/sensor/flex/2"),
            ("/device/receiver_launch", "/packet/sensor/current"),
            ("/device/receiver_launch", "/packet/sensor/voltage"),
        ],
    )
    return LaunchDescription([receiver])
