import os.path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_param_file = os.path.join(
        get_package_share_directory("power_map"),
        "config",
        "default_param.yml",
    )
    param_file_arg = DeclareLaunchArgument("param_file", default_value=default_param_file)
    return LaunchDescription(
        [
            param_file_arg,
            Node(
                package="power_map",
                executable="power-map",
                namespace="app",
                parameters=[LaunchConfiguration("param_file")],
                remappings=[("/app/power", "/device/order/power")],
            ),
        ],
    )
