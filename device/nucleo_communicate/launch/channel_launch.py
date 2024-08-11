from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    channel = Node(
        package="nucleo_communicate",
        executable="nucleo-channel",
        namespace="device",
        remappings=[
            ("/device/nucleo_state", "/packet/nucleo_state"),
            ("/device/flex_1", "/packet/sensors/flex_1"),
            ("/device/flex_2", "/packet/sensors/flex_2"),
            ("/device/current", "/packet/sensors/current"),
            ("/device/voltage", "/packet/sensors/voltage"),
            ("/device/power", "/packet/order/power"),
            ("/device/initialize", "/packet/order/initialize"),
            ("/device/suspend", "/packet/order/suspend"),
        ],
    )
    return LaunchDescription([channel])
