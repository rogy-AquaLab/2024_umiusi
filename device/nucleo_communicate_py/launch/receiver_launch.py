from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    receiver = Node(
        package="nucleo_communicate_py",
        executable="receiver",
        namespace="device",
        remappings=[
            ("/device/flex_1", "/packet/sensors/flex_1"),
            ("/device/flex_2", "/packet/sensors/flex_2"),
            ("/device/current", "/packet/sensor/current"),
            ("/device/voltage", "/packet/sensor/voltage"),
        ],
    )
    return LaunchDescription([receiver])
