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
    sender = Node(
        package="nucleo_communicate_py",
        executable="sender",
        namespace="device",
        remappings=[
            ("/device/quit", "/packet/order/quit"),
            ("/device/order/power", "/packet/order/power"),
        ],
        ros_arguments=["--log-level", log_level],
    )
    return LaunchDescription([log_level_arg, sender])
