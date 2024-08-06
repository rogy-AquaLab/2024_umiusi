from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    power_map_param_file_arg = DeclareLaunchArgument(
        "power_map_param_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("ul"), "config", "power_map_param.yml"]
        ),
    )
    power_map_param_file = LaunchConfiguration("power_map_param_file")
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("joystick"), "launch", "joystick_launch.py"]
            )
        )
    )
    app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("simple_joy_app"), "launch", "app_launch.py"]
            )
        )
    )
    power_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("power_map"), "launch", "power_map_launch.py"]
            )
        ),
        launch_arguments=[("param_file", power_map_param_file)],
    )
    return LaunchDescription([power_map_param_file_arg, joystick, app, power_map])
