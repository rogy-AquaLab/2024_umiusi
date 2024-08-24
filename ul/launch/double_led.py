from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging level for the nodes",
    )
    log_level = LaunchConfiguration("log_level")
    led_right = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("pi_led"), "launch", "led_launch.py"])
        ),
        launch_arguments=[("variant", "right")],
    )
    led_left = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("pi_led"), "launch", "led_launch.py"])
        ),
        launch_arguments=[("variant", "left")],
    )
    leds = GroupAction([led_left, led_right],
                       forwarding=False,
                       launch_configurations={
                           "log_level":log_level,
                       }
    )
    return LaunchDescription([log_level_arg, leds])
