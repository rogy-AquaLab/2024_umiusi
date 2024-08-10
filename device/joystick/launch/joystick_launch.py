from typing import Any

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def joystick_node(**kwargs: Any) -> Node:
    kwargs["package"] = "joystick"
    kwargs["executable"] = "joystick"
    kwargs["namespace"] = "device"
    return Node(**kwargs)

def generate_launch_description() -> LaunchDescription:
    index_arg = DeclareLaunchArgument("index",default_value="-1")
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug","info","warn","error","fatal"],
    )
    joystick_id_arg = DeclareLaunchArgument(
        "joystick_id",
        default_value="0",
    )
    index = LaunchConfiguration("index")
    log_level = LaunchConfiguration("log_level")

    index_specified = PythonExpression([index,">= 0"])

    indexed_joystick = joystick_node(
        name=["joystick_",index],
        remappings=[("/device/joystick","/packet/joystick")],
        ros_arguments=["--log-level",log_level],
        condition=IfCondition(index_specified),
    )

    unindexed_joystick = joystick_node(
        remappings=[("/device/joystick","/packet/joystick")],
        ros_arguments=["--log-level",log_level],
        condition=UnlessCondition(index_specified),
    )

    return LaunchDescription([
        index_arg,
        log_level_arg,
        joystick_id_arg,
        indexed_joystick,
        unindexed_joystick
    ])