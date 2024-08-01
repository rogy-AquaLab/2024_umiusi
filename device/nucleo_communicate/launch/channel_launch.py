from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    channel = Node(
        package="nucleo_communicate",
        executable="nucleo-channel",
        namespace="device",
        remappings=[
            ("/device/flex_1", "/packet/sensors/flex_1"),
            ("/device/flex_2", "/packet/sensors/flex_2"),
            ("/device/current", "/packet/sensors/current"),
            ("/device/voltage", "/packet/sensors/voltage"),
            # FIXME: /packet namespaceにremap
            ("/device/power", "/device/order/power"),
            ("/device/quit", "/device/order/quit")
        ],
    )
    return LaunchDescription([channel])
