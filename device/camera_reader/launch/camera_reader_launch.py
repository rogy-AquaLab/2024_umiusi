import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def camera_node(**kwargs: Any) -> Node:
    kwargs["package"] = "camera_reader"
    kwargs["executable"] = "camera"
    kwargs["namespace"] = "device"
    return Node(**kwargs)


def generate_launch_description() -> LaunchDescription:
    index_arg = DeclareLaunchArgument("index", default_value="-1")
    index = LaunchConfiguration("index")
    index_specified = PythonExpression([index, ">= 0"])
    default_config_path = os.path.join(
        get_package_share_directory("camera_reader"),
        "config",
        "default_param.yml",
    )
    use_index = camera_node(
        name=["camera_", index],
        parameters=[{"camera_id": index}],
        remappings=[("/device/camera_image", ["/packet/camera_image_", index])],
        condition=IfCondition(index_specified)
    )
    unuse_index = camera_node(
        parameters=[default_config_path],
        remappings=[("/device/camera_image", "/packet/camera_image")],
        condition=UnlessCondition(index_specified)
    )
    camera_reader = Node(
        package="camera_reader",
        executable="camera",
        namespace="device",
    )
    return LaunchDescription(
        [
            index_arg,
            *use_index,
            *unuse_index,
            camera_reader
        ],
    )
