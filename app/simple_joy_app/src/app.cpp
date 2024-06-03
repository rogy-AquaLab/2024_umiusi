#include <cmath>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

#include "simple_joy_app/app.hpp"

void app::App::joy_callback(const sensor_msgs::msg::Joy& msg) {
    RCLCPP_INFO_STREAM(this->get_logger(), "received joystick msg");
    // https://www.pygame.org/docs/ref/joystick.html#playstation-4-controller-pygame-2-x

    // スティックの値を見る最小範囲
    constexpr double stick_threshold = 0.1;
    // 左スティック h:水平 v:垂直
    const double lstick_h = msg.axes[0];
    const double lstick_v = -msg.axes[1]; // 上方向が負のため逆転
    // 右スティック h:水平 v:垂直
    const double rstick_h = msg.axes[2];
    const double rstick_v = -msg.axes[3];

    const bool lstick_h_effective = std::abs(lstick_h) > stick_threshold;
    const bool lstick_v_effective = std::abs(lstick_v) > stick_threshold;
    const bool lstick_effective = lstick_h_effective || lstick_v_effective;
    const bool rstick_h_effective = std::abs(rstick_h) > stick_threshold;
    const bool rstick_v_effective = std::abs(rstick_v) > stick_threshold;
    const bool rstick_effective = rstick_h_effective || rstick_v_effective;

    if (lstick_effective) {
        // TODO: 並行移動
        return;
    }
    if (!rstick_effective) {
        // TODO: 何もしない
        return;
    }
    if (rstick_h_effective) {
        // TODO: 水平回転
        return;
    }
    // assert(rstick_v_effective); 自明
    // TODO: 垂直方向の移動
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
