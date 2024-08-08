from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
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
    leds = GroupAction([led_left, led_right], forwarding=False)
    return LaunchDescription([leds])
