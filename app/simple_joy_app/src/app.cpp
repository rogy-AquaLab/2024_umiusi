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
    const bool lstick_effective   = lstick_h_effective || lstick_v_effective;
    const bool rstick_h_effective = std::abs(rstick_h) > stick_threshold;
    const bool rstick_v_effective = std::abs(rstick_v) > stick_threshold;
    const bool rstick_effective   = rstick_h_effective || rstick_v_effective;

    if (lstick_effective) {
        packet_interfaces::msg::Power msg = this->para_move_power({ lstick_v, lstick_h });
        this->power_publisher->publish(msg);
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

auto app::App::para_move_power(const std::pair<double, double>& stick
) -> packet_interfaces::msg::Power {
    // FIXME: C++20...?
    constexpr double pi = 3.141592653589793;
    // FIXME: use parameter
    constexpr uint16_t bldc_center = 1480;
    constexpr uint16_t bldc_radius = 250; // TODO: check this
    constexpr uint16_t servo_min   = 500;
    constexpr uint16_t servo_max   = 2400;
    // TODO: provide explanation
    constexpr double theta = pi / 4;
    // Left-Up, Right-Down
    double bldc1 = stick.first * cos(theta) + stick.second * sin(theta);
    // Right-Up, Left-Down
    double       bldc2        = stick.first * cos(theta) - stick.second * sin(theta);
    const double bldc_abc_max = std::max(std::abs(bldc1), std::abs(bldc2));
    // constrain
    if (bldc_abc_max > 1) {
        const double scale = 1 / bldc_abc_max;
        bldc1 *= scale;
        bldc2 *= scale;
    }

    packet_interfaces::msg::Power msg{};

    const uint16_t bldc1_msg = static_cast<uint16_t>(
        static_cast<double>(bldc_center) + static_cast<double>(bldc_radius) * bldc1
    );
    const uint16_t bldc2_msg = static_cast<uint16_t>(
        static_cast<double>(bldc_center) + static_cast<double>(bldc_radius) * bldc2
    );

    // TODO: Fix order with param
    msg.bldc[0] = bldc1_msg;
    msg.bldc[1] = bldc2_msg;
    msg.bldc[2] = bldc2_msg;
    msg.bldc[3] = bldc1_msg;

    constexpr uint16_t servo_center = (servo_min + servo_max) / 2;

    msg.servo[0] = servo_center;
    msg.servo[1] = servo_center;
    msg.servo[2] = servo_center;
    msg.servo[3] = servo_center;

    return msg;
}

app::App::App(const rclcpp::NodeOptions& options) :
    rclcpp::Node("simple_joy_app", options),
    subscription(),
    power_publisher() {
    using std::placeholders::_1;
    auto callback      = std::bind(&app::App::joy_callback, this, _1);
    this->subscription = this->create_subscription<sensor_msgs::msg::Joy>(
        "/device/joystick", 10, callback
    );
    this->power_publisher = this->create_publisher<packet_interfaces::msg::Power>(
        "/device/order/power", 10
    );
}

RCLCPP_COMPONENTS_REGISTER_NODE(app::App)
