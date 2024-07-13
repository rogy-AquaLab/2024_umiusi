from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="simple_joy_app",
                executable="app",
                namespace="app",
                remappings=[("/app/joystick", "/packet/joystick")],
            ),
        ],
    )
