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
        description="Logging level for the nodes"
    )
    log_level = LaunchConfiguration("log_level")
    camera0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_reader"), "launch", "camera_reader_launch.py"]
            )
        ),
        launch_arguments=[("index", "0"),("log_level", log_level)],
    )
    camera1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_reader"), "launch", "camera_reader_launch.py"]
            )
        ),
        launch_arguments=[("index", "1"),("log_level", log_level)],
    )
    cameras = GroupAction([camera0, camera1], forwarding=False)
    return LaunchDescription([log_level_arg,cameras])
