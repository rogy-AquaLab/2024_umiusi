from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
    )
    log_level = LaunchConfiguration("log_level")
    main = Node(
        package="nucleo_communicate_py",
        executable="channel",
        namespace="device",
        remappings=[
            ("/device/flex_1", "/packet/sensors/flex_1"),
            ("/device/flex_2", "/packet/sensors/flex_2"),
            ("/device/current", "/packet/sensor/current"),
            ("/device/voltage", "/packet/sensor/voltage"),
            ("/device/quit", "/packet/order/quit"),
            ("/device/order/power", "/packet/order/power"),
        ],
        ros_arguments=["--log-level", log_level],
    )
    return LaunchDescription([log_level_arg, main])
