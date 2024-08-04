from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    camera0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_reader"), "launch", "camera_reader_launch.py"]
            )
        ),
        launch_arguments=[("index", "0")],
    )
    camera1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_reader"), "launch", "camera_reader_launch.py"]
            )
        ),
        launch_arguments=[("index", "1")],
    )
    return LaunchDescription([camera0, camera1])
