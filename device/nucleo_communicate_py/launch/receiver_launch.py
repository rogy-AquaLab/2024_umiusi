from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging level for the nodes",
    )
    log_level = LaunchConfiguration("log_level")
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
        ros_arguments=["--log-level", log_level],
    )
    return LaunchDescription([log_level_arg, receiver])
