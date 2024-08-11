from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # args
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        choices=["debug", "info", "warn", "error", "fatal"],
        description="Logging level for the nodes",
    )
    log_level = LaunchConfiguration("log_level")
    use_nucleo_arg = DeclareLaunchArgument(
        "use_nucleo", default_value="true", choices=["true", "false"]
    )
    camera_variant_arg = DeclareLaunchArgument(
        "camera_variant", default_value="single", choices=["none", "single", "double"]
    )
    use_led_arg = DeclareLaunchArgument(
        # TODO: change default value to true
        "use_led",
        default_value="false",
        choices=["true", "false"],
    )
    use_imu_arg = DeclareLaunchArgument(
        # TODO: change default value to true
        "use_imu",
        default_value="false",
        choices=["true", "false"],
    )
    args = GroupAction(
        [log_level_arg, use_nucleo_arg, camera_variant_arg, use_led_arg, use_imu_arg],
        scoped=False,
    )
    # substitutions
    use_nucleo = LaunchConfiguration("use_nucleo")
    camera_variant = LaunchConfiguration("camera_variant")
    use_single_camera = PythonExpression(["'", camera_variant, "' == 'single'"])
    use_double_camera = PythonExpression(["'", camera_variant, "' == 'double'"])
    use_led = LaunchConfiguration("use_led")
    use_imu = LaunchConfiguration("use_imu")
    # nodes
    nucleo_channel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ul"), "launch", "nucleo_channel.py"])
        ),
        launch_arguments=[("log_level", log_level)],
        condition=IfCondition(use_nucleo),
    )
    single_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_reader"), "launch", "camera_reader_launch.py"]
            )
        ),
        launch_arguments=[("log_level", log_level)],
        condition=IfCondition(use_single_camera),
    )
    double_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_reader"), "launch", "camera_reader_launch.py"]
            )
        ),
        launch_arguments=[("log_level", log_level)],
        condition=IfCondition(use_double_camera),
    )
    led = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ul"), "launch", "double_led.py"]
                    )
                ),
                launch_arguments=[("log_level", log_level)],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/packet/order/led_color_left",
                    "packet_interfaces/msg/LedColor",
                    "{red: true, blue: false, green: false}",
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/packet/order/led_color_right",
                    "packet_interfaces/msg/LedColor",
                    "{red: true, blue: false, green: false}",
                ]
            ),
        ],
        condition=IfCondition(use_led),
    )
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("pi_i2c"), "launch", "imu_launch.py"])
        ),
        launch_arguments=[("log_level", log_level)],
        condition=IfCondition(use_imu),
    )
    nodes = GroupAction(
        [nucleo_channel, single_camera, double_camera, led, imu],
        forwarding=False,
        launch_configurations={
            "use_nucleo": use_nucleo,
            "camera_variant": camera_variant,
            "use_led": use_led,
            "use_imu": use_imu,
        },
    )

    return LaunchDescription([args, nodes])
