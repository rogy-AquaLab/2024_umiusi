#ifndef SIMPLE_JOY_APP_APP_HPP
#define SIMPLE_JOY_APP_APP_HPP

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <power_map_msg/msg/normalized_power.hpp>

namespace app {

class App : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;

    rclcpp::Publisher<power_map_msg::msg::NormalizedPower>::SharedPtr power_publisher;

    void joy_callback(const sensor_msgs::msg::Joy& msg);

    auto para_move_power(const std::pair<double, double>& stick
    ) -> power_map_msg::msg::NormalizedPower;

    auto rotate_power(const double& hstick) -> power_map_msg::msg::NormalizedPower;

    auto stop_power() -> power_map_msg::msg::NormalizedPower;

public:
    App(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // SIMPLE_JOY_APP_APP_HPP
