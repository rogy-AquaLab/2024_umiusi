from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    main = Node(
        package="nucleo_communicate_py",
        executable="channel",
        namespace="device",
        remappings=[
            ("/device/flex_1", "/packet/sensors/flex_1"),
            ("/device/flex_2", "/packet/sensors/flex_2"),
            ("/device/current", "/packet/sensor/current"),
            ("/device/voltage", "/packet/sensor/voltage"),
        ]
    )
    return LaunchDescription([main])
