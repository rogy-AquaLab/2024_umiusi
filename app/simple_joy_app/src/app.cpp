#include <chrono>
#include <cmath>
#include <functional>
#include <numbers>

#include "rclcpp_components/register_node_macro.hpp"

#include "simple_joy_app/app.hpp"

using power_map_msg::msg::NormalizedPower;

auto app::App::generate_header() -> std_msgs::msg::Header {
    std_msgs::msg::Header header;
    header.frame_id = "simple_joy_app";
    header.stamp    = this->get_clock()->now();
    return header;
}

void app::App::publish_power(power_map_msg::msg::NormalizedPower& msg) {
    msg.header = this->generate_header();
    this->power_publisher->publish(msg);
}

bool power_is_zero(const NormalizedPower& power) {
    for (float b : power.bldc) {
        if (std::abs(b) > 0.001) {
            return false;
        }
    }
    return true;
}

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

    NormalizedPower pub_msg;
    if (lstick_effective) {
        pub_msg = this->para_move_power({ lstick_v, lstick_h });
    } else if (!rstick_effective) {
        pub_msg = this->stop_power();
    } else if (rstick_h_effective) {
        pub_msg = this->rotate_power(rstick_h);
    } else {
        // assert(rstick_v_effective); 自明
        pub_msg = this->vertical_move_power(rstick_v);
    }
    this->status = power_is_zero(pub_msg) ? Status::Stopped : Status::Moving;
    this->publish_power(pub_msg);
}

auto app::App::para_move_power(const std::pair<double, double>& stick
) -> NormalizedPower {
    // TODO: provide explanation
    constexpr double theta = std::numbers::pi_v<double> / 4;
    // Left-Up, Right-Down
    float bldc1 = stick.first * cos(theta) + stick.second * sin(theta);
    // Right-Up, Left-Down
    float bldc2 = stick.first * cos(theta) - stick.second * sin(theta);

    const double bldc_abs_max = std::max(std::abs(bldc1), std::abs(bldc2));
    if (bldc_abs_max > 1) {
        const double scale = 1 / bldc_abs_max;
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
    msg.bldc.fill(static_cast<float>(mag));
    msg.servo.fill(static_cast<float>(sign * std::numbers::pi_v<double>));

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

void app::App::healthcheck() {
    using packet_interfaces::msg::LedColor;
    constexpr auto color_red = [](LedColor& color) {
        color.red   = true;
        color.green = false;
        color.blue  = false;
    };
    constexpr auto color_green = [](LedColor& color) {
        color.red   = false;
        color.green = true;
        color.blue  = false;
    };
    constexpr auto color_blue = [](LedColor& color) {
        color.red   = false;
        color.green = false;
        color.blue  = true;
    };

    LedColor left_msg, right_msg;
    left_msg.header  = this->generate_header();
    right_msg.header = this->generate_header();
    switch (this->status) {
    case Status::NoInput:
        color_blue(left_msg);
        color_red(right_msg);
        break;
    case Status::Moving:
        color_green(left_msg);
        color_blue(right_msg);
        break;
    case Status::Stopped:
        color_green(left_msg);
        color_green(right_msg);
        break;
    }
    this->led_left_publisher->publish(left_msg);
    this->led_right_publisher->publish(right_msg);
    this->status = Status::NoInput;
}

app::App::App(const rclcpp::NodeOptions& options) :
    rclcpp::Node("simple_joy_app", options),
    subscription(),
    power_publisher(),
    led_left_publisher(),
    led_right_publisher(),
    healthcheck_timer(),
    status(app::Status::NoInput) {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    auto joy_callback = std::bind(&app::App::joy_callback, this, _1);
    this->subscription
        = this->create_subscription<sensor_msgs::msg::Joy>("joystick", 10, joy_callback);
    this->power_publisher
        = this->create_publisher<NormalizedPower>("normalized_power", 10);
    this->led_left_publisher
        = this->create_publisher<packet_interfaces::msg::LedColor>("led_color_left", 10);
    this->led_right_publisher
        = this->create_publisher<packet_interfaces::msg::LedColor>("led_color_right", 10);
    auto healthcheck_callback = std::bind(&app::App::healthcheck, this);
    this->healthcheck_timer   = this->create_wall_timer(100ms, healthcheck_callback);
}

RCLCPP_COMPONENTS_REGISTER_NODE(app::App)
