from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    imshow0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("imshow"), "launch", "imshow_launch.py"]
            )
        ),
        launch_arguments=[("index", "0")],
    )
    imshow1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("imshow"), "launch", "imshow_launch.py"]
            )
        ),
        launch_arguments=[("index", "1")],
    )
    imshows = GroupAction([imshow0, imshow1], forwarding=False)
    return LaunchDescription([imshows])
