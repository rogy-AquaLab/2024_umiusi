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
    index = LaunchConfiguration("index")
    index_specified = PythonExpression([index, ">= 0"])
    indexed_imshow = imshow_node(
        name=["imshow_", index],
        remappings=[("/app/camera_image", ["/packet/camera_image_", index])],
        condition=IfCondition(index_specified),
    )
    unindexed_imshow = imshow_node(
        remappings=[("/app/camera_image", "/packet/camera_image")],
        condition=UnlessCondition(index_specified),
    )
    return LaunchDescription(
        [index_arg, indexed_imshow, unindexed_imshow],
    )
