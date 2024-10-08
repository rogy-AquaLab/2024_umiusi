#ifndef SIMPLE_JOY_APP_APP_HPP
#define SIMPLE_JOY_APP_APP_HPP

#include <optional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/header.hpp>

#include <packet_interfaces/msg/led_color.hpp>
#include <packet_interfaces/msg/nucleo_state.hpp>
#include <power_map_msg/msg/normalized_power.hpp>

#include "simple_joy_app/status.hpp"

namespace app {

class App : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    rclcpp::Subscription<packet_interfaces::msg::NucleoState>::SharedPtr
        nucleo_state_subscription;

    rclcpp::Publisher<power_map_msg::msg::NormalizedPower>::SharedPtr power_publisher;

    rclcpp::Publisher<packet_interfaces::msg::LedColor>::SharedPtr led_left_publisher;
    rclcpp::Publisher<packet_interfaces::msg::LedColor>::SharedPtr led_right_publisher;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr initialize_publisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr suspend_publisher;

    rclcpp::TimerBase::SharedPtr healthcheck_timer;

    app::Status      status;
    app::NucleoState nucleo_state;

    std::optional<rclcpp::Time> vertical_move_start_at;

    auto generate_header() -> std_msgs::msg::Header;

    // headerを修飾してpublish
    void publish_power(power_map_msg::msg::NormalizedPower& msg);

    void joy_callback(const sensor_msgs::msg::Joy& msg);

    void nucleo_state_callback(const packet_interfaces::msg::NucleoState& msg);

    auto para_move_power(const std::pair<double, double>& stick
    ) -> power_map_msg::msg::NormalizedPower;

    auto rotate_power(const double& hstick) -> power_map_msg::msg::NormalizedPower;

    auto vertical_move_power(const double& vstick) -> power_map_msg::msg::NormalizedPower;

    auto stop_power() -> power_map_msg::msg::NormalizedPower;

    void healthcheck();

public:
    App(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // SIMPLE_JOY_APP_APP_HPP
