#include <chrono>
#include <cmath>
#include <functional>
#include <numbers>

#include "rclcpp_components/register_node_macro.hpp"

#include "simple_joy_app/app.hpp"

using power_map_msg::msg::NormalizedPower;
using namespace std::chrono_literals;

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

    const bool rbutton_left       = msg.buttons[0];
    const bool rbutton_down       = msg.buttons[1];
    const bool lstick_h_effective = std::abs(lstick_h) > stick_threshold;
    const bool lstick_v_effective = std::abs(lstick_v) > stick_threshold;
    const bool lstick_effective   = lstick_h_effective || lstick_v_effective;
    const bool rstick_h_effective = std::abs(rstick_h) > stick_threshold;
    const bool rstick_v_effective = std::abs(rstick_v) > stick_threshold;
    const bool rstick_effective   = rstick_h_effective || rstick_v_effective;

    if (rbutton_left) {
        std_msgs::msg::Empty pub_msg;
        this->initialize_publisher->publish(pub_msg);
    } else if (rbutton_down) {
        std_msgs::msg::Empty pub_msg;
        this->suspend_publisher->publish(pub_msg);
    }

    NormalizedPower pub_msg;
    if (lstick_effective) {
        this->vertical_move_start_at = std::nullopt;
        pub_msg                      = this->para_move_power({ lstick_v, lstick_h });
    } else if (!rstick_effective) {
        this->vertical_move_start_at = std::nullopt;
        pub_msg                      = this->stop_power();
    } else if (rstick_h_effective) {
        this->vertical_move_start_at = std::nullopt;
        pub_msg                      = this->rotate_power(rstick_h);
    } else {
        // assert(rstick_v_effective); 自明
        if (!this->vertical_move_start_at.has_value()) {
            this->vertical_move_start_at = this->get_clock()->now();
        }
        pub_msg = this->vertical_move_power(rstick_v);
    }
    this->status = power_is_zero(pub_msg) ? Status::Stopped : Status::Moving;
    this->publish_power(pub_msg);
}

void app::App::nucleo_state_callback(const packet_interfaces::msg::NucleoState& msg) {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "received nucleo_state msg");
    this->nucleo_state = static_cast<NucleoState>(msg.state);
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
    const auto duration_rclcpp = this->get_clock()->now()
                                 - this->vertical_move_start_at.value();
    const auto duration    = duration_rclcpp.to_chrono<std::chrono::milliseconds>();
    const bool affect_bldc = duration > 500ms;

    const double mag  = std::abs(vstick) * (affect_bldc ? 1.0 : 0.0);
    const double sign = std::signbit(vstick) ? -1 : 1;

    power_map_msg::msg::NormalizedPower msg{};
    // FIXME: tekito- ni kimeta
    msg.bldc.fill(static_cast<float>(mag));
    msg.servo.fill(static_cast<float>(sign * std::numbers::pi_v<double>));

    return msg;
}

auto app::App::rotate_power(const double& hstick) -> power_map_msg::msg::NormalizedPower {
    const double mag  = std::abs(hstick) * 0.7;
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

    RCLCPP_DEBUG_STREAM(
        this->get_logger(), "simple_joy_app status: " << status_str(this->status)
    );
    LedColor left_msg, right_msg;
    left_msg.header = this->generate_header();
    switch (this->nucleo_state) {
    case NucleoState::Suspend:      color_red(left_msg); break;
    case NucleoState::Initializing: color_blue(left_msg); break;
    case NucleoState::Running:      color_green(left_msg); break;
    }
    right_msg.header = this->generate_header();
    switch (this->status) {
    case Status::NoInput: color_red(right_msg); break;
    case Status::Stopped: color_blue(right_msg); break;
    case Status::Moving:  color_green(right_msg); break;
    }
    this->led_left_publisher->publish(left_msg);
    this->led_right_publisher->publish(right_msg);
    this->status = Status::NoInput;
}

app::App::App(const rclcpp::NodeOptions& options) :
    rclcpp::Node("simple_joy_app", options),
    joy_subscription(),
    nucleo_state_subscription(),
    power_publisher(),
    led_left_publisher(),
    led_right_publisher(),
    initialize_publisher(),
    suspend_publisher(),
    healthcheck_timer(),
    status(app::Status::NoInput),
    nucleo_state(app::NucleoState::Suspend),
    vertical_move_start_at(std::nullopt) {
    using namespace std::chrono_literals;
    using std::placeholders::_1;
    auto joy_callback = std::bind(&app::App::joy_callback, this, _1);
    this->joy_subscription
        = this->create_subscription<sensor_msgs::msg::Joy>("joystick", 10, joy_callback);
    auto ns_callback = std::bind(&app::App::nucleo_state_callback, this, _1);
    this->nucleo_state_subscription
        = this->create_subscription<packet_interfaces::msg::NucleoState>(
            "nucleo_state", 10, ns_callback
        );
    this->power_publisher
        = this->create_publisher<NormalizedPower>("normalized_power", 10);
    this->led_left_publisher
        = this->create_publisher<packet_interfaces::msg::LedColor>("led_color_left", 10);
    this->led_right_publisher
        = this->create_publisher<packet_interfaces::msg::LedColor>("led_color_right", 10);
    this->initialize_publisher
        = this->create_publisher<std_msgs::msg::Empty>("order/initialize", 10);
    this->suspend_publisher
        = this->create_publisher<std_msgs::msg::Empty>("order/suspend", 10);
    auto healthcheck_callback = std::bind(&app::App::healthcheck, this);
    this->healthcheck_timer   = this->create_wall_timer(100ms, healthcheck_callback);
}

RCLCPP_COMPONENTS_REGISTER_NODE(app::App)
