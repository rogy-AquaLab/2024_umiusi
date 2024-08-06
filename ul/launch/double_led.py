from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
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
    return LaunchDescription([led_right, led_left])
