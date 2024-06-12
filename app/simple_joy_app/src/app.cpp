#include <cmath>
#include <functional>

#include "rclcpp_components/register_node_macro.hpp"

#include "simple_joy_app/app.hpp"

using power_map_msg::msg::NormalizedPower;

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
        NormalizedPower msg = this->para_move_power({ lstick_v, lstick_h });
        this->power_publisher->publish(msg);
        return;
    }
    if (!rstick_effective) {
        NormalizedPower msg = this->stop_power();
        this->power_publisher->publish(msg);
        return;
    }
    if (rstick_h_effective) {
        NormalizedPower msg = this->rotate_power(rstick_h);
        this->power_publisher->publish(msg);
        return;
    }
    // assert(rstick_v_effective); 自明
    this->power_publisher->publish(this->vertical_move_power(rstick_v));
}

auto app::App::para_move_power(const std::pair<double, double>& stick
) -> NormalizedPower {
    // FIXME: C++20...?
    constexpr double pi = 3.141592653589793;
    // TODO: provide explanation
    constexpr double theta = pi / 4;
    // Left-Up, Right-Down
    float bldc1 = stick.first * cos(theta) + stick.second * sin(theta);
    // Right-Up, Left-Down
    float bldc2 = stick.first * cos(theta) - stick.second * sin(theta);

    const double bldc_abc_max = std::max(std::abs(bldc1), std::abs(bldc2));
    if (bldc_abc_max > 1) {
        const double scale = 1 / bldc_abc_max;
        bldc1 *= scale;
        bldc2 *= scale;
    }

    NormalizedPower msg{};

    msg.bldc[0] = bldc1;
    msg.bldc[1] = bldc2;
    msg.bldc[2] = bldc2;
    msg.bldc[3] = bldc1;

    msg.servo[0] = 0.0;
    msg.servo[1] = 0.0;
    msg.servo[2] = 0.0;
    msg.servo[3] = 0.0;

    return msg;
}

auto app::App::vertical_move_power(const double& vstick
) -> power_map_msg::msg::NormalizedPower {
    const double mag  = std::abs(vstick);
    const double sign = std::signbit(vstick) ? -1 : 1;

    power_map_msg::msg::NormalizedPower msg{};
    // FIXME: tekito- ni kimeta
    msg.bldc[0] = mag;
    msg.bldc[1] = mag;
    msg.bldc[2] = mag;
    msg.bldc[3] = mag;

    msg.servo[0] = sign;
    msg.servo[1] = sign;
    msg.servo[2] = sign;
    msg.servo[3] = sign;

    return msg;
}

auto app::App::rotate_power(const double& hstick) -> power_map_msg::msg::NormalizedPower {
    const double mag  = std::abs(hstick);
    const double sign = std::signbit(hstick) ? -1 : 1;

    NormalizedPower msg{};

    msg.bldc[0] = sign * mag;
    msg.bldc[1] = -sign * mag;
    msg.bldc[2] = sign * mag;
    msg.bldc[3] = -sign * mag;

    msg.servo[0] = 0.0;
    msg.servo[1] = 0.0;
    msg.servo[2] = 0.0;
    msg.servo[3] = 0.0;

    return msg;
}

auto app::App::stop_power() -> NormalizedPower {
    NormalizedPower msg{};

    msg.bldc[0] = 0.0;
    msg.bldc[1] = 0.0;
    msg.bldc[2] = 0.0;
    msg.bldc[3] = 0.0;

    msg.servo[0] = 0.0;
    msg.servo[1] = 0.0;
    msg.servo[2] = 0.0;
    msg.servo[3] = 0.0;

    return msg;
}

app::App::App(const rclcpp::NodeOptions& options) :
    rclcpp::Node("simple_joy_app", options),
    subscription(),
    power_publisher() {
    using std::placeholders::_1;
    auto callback = std::bind(&app::App::joy_callback, this, _1);
    this->subscription
        = this->create_subscription<sensor_msgs::msg::Joy>("joystick", 10, callback);
    this->power_publisher
        = this->create_publisher<NormalizedPower>("normalized_power", 10);
}

RCLCPP_COMPONENTS_REGISTER_NODE(app::App)
