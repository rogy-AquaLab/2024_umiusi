from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
    )
    log_level = LaunchConfiguration("log_level")
    app = Node(
        package="simple_joy_app",
        executable="app",
        namespace="app",
        remappings=[
            ("/app/joystick", "/packet/joystick"),
            ("/app/led_color_left", "/packet/order/led_color_left"),
            ("/app/led_color_right", "/packet/order/led_color_right"),
        ],
        ros_arguments=["--log-level", log_level],
    )
    return LaunchDescription([log_level_arg, app])
