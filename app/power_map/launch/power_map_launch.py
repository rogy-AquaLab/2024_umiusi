from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    param_file = PathJoinSubstitution(
        [FindPackageShare("power_map"), "config", "default_param.yml"]
    )
    param_file_arg = DeclareLaunchArgument("param_file", default_value=param_file)
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
    )
    log_level = LaunchConfiguration("log_level")
    power_map = Node(
        package="power_map",
        executable="power-map",
        namespace="app",
        parameters=[LaunchConfiguration("param_file")],
        remappings=[("/app/power", "/device/order/power")],
        ros_arguments=["--log-level", log_level],
    )
    return LaunchDescription([param_file_arg, log_level_arg, power_map])
