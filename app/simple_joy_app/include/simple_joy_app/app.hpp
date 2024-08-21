#ifndef SIMPLE_JOY_APP_APP_HPP
#define SIMPLE_JOY_APP_APP_HPP

#include <optional>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/header.hpp>

#include <packet_interfaces/msg/led_color.hpp>
#include <power_map_msg/msg/normalized_power.hpp>

#include "simple_joy_app/status.hpp"

namespace app {

class App : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;

    rclcpp::Publisher<power_map_msg::msg::NormalizedPower>::SharedPtr power_publisher;

    rclcpp::Publisher<packet_interfaces::msg::LedColor>::SharedPtr led_left_publisher;
    rclcpp::Publisher<packet_interfaces::msg::LedColor>::SharedPtr led_right_publisher;

    rclcpp::TimerBase::SharedPtr healthcheck_timer;

    app::Status status;

    std::optional<rclcpp::Time> vertical_move_start_at;

    auto generate_header() -> std_msgs::msg::Header;

    // headerを修飾してpublish
    void publish_power(power_map_msg::msg::NormalizedPower& msg);

    void joy_callback(const sensor_msgs::msg::Joy& msg);

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
