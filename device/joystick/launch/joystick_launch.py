from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="joystick",
            executable="joystick",
            namespace="device",
            remappings=[("/device/joystick", "/packet/joystick")]
        )
    ])
