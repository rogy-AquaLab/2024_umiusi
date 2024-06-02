#include <functional>

#include "rclcpp_components/register_node_macro.hpp"
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>

#include "simple_joy_app/app.hpp"

void app::App::joy_callback(const sensor_msgs::msg::Joy& msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "received joystick msg");
    // https://www.pygame.org/docs/ref/joystick.html#playstation-4-controller-pygame-2-x
    // TODO
}

app::App::App(const rclcpp::NodeOptions& options) :
    rclcpp::Node("simple_joy_app", options),
    subscription() {
    using std::placeholders::_1;
    auto callback      = std::bind(&app::App::joy_callback, this, _1);
    this->subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "/device/joystick", 10, callback
    );
}

RCLCPP_COMPONENTS_REGISTER_NODE(app::App)
