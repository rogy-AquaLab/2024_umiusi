from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def camera_node(**kwargs: Any) -> Node:
    kwargs["package"] = "camera_reader"
    kwargs["executable"] = "camera"
    kwargs["namespace"] = "device"
    return Node(**kwargs)


def generate_launch_description() -> LaunchDescription:
    index_arg = DeclareLaunchArgument("index", default_value="-1")
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging level for the nodes",
    )
    log_level = LaunchConfiguration("log_level")
    index = LaunchConfiguration("index")
    index_specified = PythonExpression([index, ">= 0"])
    default_config_path = PathJoinSubstitution(
        [FindPackageShare("camera_reader"), "config", "default_param.yml"]
    )
    use_index = camera_node(
        name=["camera_", index],
        parameters=[{"camera_id": index}],
        remappings=[("/device/camera_image", ["/packet/camera_image_", index])],
        ros_arguments=["--log-level", log_level],
        condition=IfCondition(index_specified),
    )
    unuse_index = camera_node(
        parameters=[default_config_path],
        remappings=[("/device/camera_image", "/packet/camera_image")],
        ros_arguments=["--log-level", log_level],
        condition=UnlessCondition(index_specified),
    )
    return LaunchDescription(
        [index_arg, log_level_arg,use_index, unuse_index],
    )
