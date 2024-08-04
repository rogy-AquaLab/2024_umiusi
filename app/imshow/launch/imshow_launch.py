from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap


def generate_launch_description() -> LaunchDescription:
    index_arg = DeclareLaunchArgument("index", default_value="-1")
    index = LaunchConfiguration("index")
    index_specified = PythonExpression([index, ">= 0"])
    imshow = Node(
        package="imshow",
        executable="imshow",
        namespace="app",
    )
    return LaunchDescription(
        [
            index_arg,
            SetRemap(
                "/app/camera_image", ["/packet/camera_image_", index],
                condition=IfCondition(index_specified)
            ),
            SetRemap(
                "/app/camera_image", "/packet/camera_image",
                condition=UnlessCondition(index_specified)
            ),
            imshow,
        ],
    )
