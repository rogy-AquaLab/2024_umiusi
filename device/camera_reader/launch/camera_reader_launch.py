import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter, SetParametersFromFile, SetRemap


def generate_launch_description() -> LaunchDescription:
    index_arg = DeclareLaunchArgument("index", default_value="-1")
    index = LaunchConfiguration("index")
    index_specified = PythonExpression([index, ">= 0"])
    default_config_path = os.path.join(
        get_package_share_directory("camera_reader"),
        "config",
        "default_param.yml",
    )
    use_index = [
        SetParameter(
            "camera_id", index,
            condition=IfCondition(index_specified)
        ),
        SetRemap(
            "/device/camera_image", ["/packet/camera_image_", index],
            condition=IfCondition(index_specified)
        ),
    ]
    unuse_index = [
        SetParametersFromFile(
            default_config_path,
            condition=UnlessCondition(index_specified)
        ),
        SetRemap(
            "/device/camera_image", "/packet/camera_image",
            condition=UnlessCondition(index_specified)
        ),
    ]
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
