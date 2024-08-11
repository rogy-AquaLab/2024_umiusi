from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:
    state_arg = DeclareLaunchArgument(
        "state", default_value="running", choices=["initializing", "suspend", "running"]
    )
    power_topic_arg = DeclareLaunchArgument("power_topic", default_value="{}")
    # launch_ros.actions.Nodeを使っていないためlog_level引数はない
    state = LaunchConfiguration("state")
    power_topic = LaunchConfiguration("power_topic")
    state_is_initializing = PythonExpression(["'", state, "' == 'initializing'"])
    state_is_suspend = PythonExpression(["'", state, "' == 'suspend'"])
    state_is_running = PythonExpression(["'", state, "' == 'running'"])
    order_power = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/packet/order/power",
            "packet_interfaces/msg/Power",
            power_topic,
        ],
        condition=IfCondition(state_is_running),
    )
    order_initializing = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/packet/order/initialize",
            "std_msgs/msg/Empty",
            "{}",
        ],
        condition=IfCondition(state_is_initializing),
    )
    order_suspend = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "--once",
            "/packet/order/suspend",
            "std_msgs/msg/Empty",
            "{}",
        ],
        condition=IfCondition(state_is_suspend),
    )
    return LaunchDescription(
        [state_arg, power_topic_arg, order_power, order_initializing, order_suspend]
    )
