from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def imshow_node(**kwargs: Any) -> Node:
    kwargs["package"] = "imshow"
    kwargs["executable"] = "imshow"
    kwargs["namespace"] = "app"
    return Node(**kwargs)


def generate_launch_description() -> LaunchDescription:
    index_arg = DeclareLaunchArgument("index", default_value="-1")
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
    )
    index = LaunchConfiguration("index")
    log_level = LaunchConfiguration("log_level")
    index_specified = PythonExpression([index, ">= 0"])
    indexed_imshow = imshow_node(
        name=["imshow_", index],
        remappings=[("/app/camera_image", ["/packet/camera_image_", index])],
        ros_arguments=["--log-level", log_level],
        condition=IfCondition(index_specified),
    )
    unindexed_imshow = imshow_node(
        remappings=[("/app/camera_image", "/packet/camera_image")],
        ros_arguments=["--log-level", log_level],
        condition=UnlessCondition(index_specified),
    )
    return LaunchDescription([index_arg, log_level_arg, indexed_imshow, unindexed_imshow])
