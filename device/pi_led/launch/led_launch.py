import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def led_node(**kwargs: Any) -> Node:
    kwargs["package"] = "pi_led"
    kwargs["executable"] = "pi_led"
    kwargs["namespace"] = "device"
    return Node(**kwargs)


def generate_launch_description() -> LaunchDescription:
    # arguments
    variant_arg = DeclareLaunchArgument(
        "variant",
        default_value="default",
        choices=["default", "right", "left", "custom"]
    )
    param_file_arg = DeclareLaunchArgument("param_file", default_value="")
    # substitutions
    variant = LaunchConfiguration("variant")
    param_file = LaunchConfiguration("param_file")
    variant_is_default = PythonExpression([variant, "== 'default'"])
    variant_is_right = PythonExpression([variant, "== 'right'"])
    variant_is_left = PythonExpression([variant, "== 'left'"])
    variant_is_custom = PythonExpression([variant, "== 'custom'"])
    # file paths
    right_param_file = os.path.join(
        get_package_share_directory("pi_led"),
        "config",
        "default_param.yml",
    )
    left_param_file = os.path.join(
        get_package_share_directory("pi_led"),
        "config",
        "left_param.yml",
    )
    # exec actions
    use_default = led_node(
        parameters=[right_param_file],
        remappings=[("/device/led_color", "/packet/order/led_color")],
        condition=IfCondition(variant_is_default)
    )
    use_right = led_node(
        name="led_right",
        parameters=[right_param_file],
        remappings=[("/device/led_color", "/packet/order/led_color_right")],
        condition=IfCondition(variant_is_right)
    )
    use_left = led_node(
        name="led_left",
        parameters=[left_param_file],
        remappings=[("/device/led_color", "/packet/order/led_color_left")],
        condition=IfCondition(variant_is_left)
    )
    use_custom = led_node(
        name="led_custom",
        parameters=[param_file],
        remappings=[("/device/led_color", "/packet/order/led_color_custom")],
        condition=IfCondition(variant_is_custom)
    )

    return LaunchDescription([
        variant_arg,
        param_file_arg,
        use_default,
        use_right,
        use_left,
        use_custom
    ])
