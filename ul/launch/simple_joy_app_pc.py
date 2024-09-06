from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level for the nodes",
    )
    log_level = LaunchConfiguration("log-level")
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
        ),
        launch_arguments=[("log-level", log_level)],
    )
    app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("simple_joy_app"), "launch", "app_launch.py"]
            )
        ),
        launch_arguments=[("log-level", log_level)],
    )
    power_map = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("power_map"), "launch", "power_map_launch.py"]
            )
        ),
        launch_arguments=[("param_file", power_map_param_file)],
    )
    nodes = GroupAction(
        [joystick, app, power_map],
        forwarding=False,
        launch_configurations={
            "param_file": power_map_param_file,
            "power_map_param_file": power_map_param_file,
        },
    )
    return LaunchDescription([log_level_arg,power_map_param_file_arg, nodes])
