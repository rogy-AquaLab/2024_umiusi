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
    all_exec = Node(
        package="pi_i2c",
        executable="all",
        namespace="device",
        remappings=[("/device/imu", "/packet/sensors/imu")],
        ros_arguments=["--log-level", log_level],
    )
    return LaunchDescription([log_level_arg, all_exec])
