from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def joystick_node(**kwargs: Any) -> Node:
    kwargs["package"] = "joystick"
    kwargs["executable"] = "joystick"
    kwargs["namespace"] = "device"
    return Node(**kwargs)


def generate_launch_description() -> LaunchDescription:
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
    )
    log_level = LaunchConfiguration("log_level")

    joystick = Node(
        package="joystick",
        executable="joystick",
        namespace="device",
        remappings=[("/device/joystick", "/packet/joystick")],
        ros_arguments=["--log-level", log_level],
    )

    return LaunchDescription([log_level_arg, joystick])
