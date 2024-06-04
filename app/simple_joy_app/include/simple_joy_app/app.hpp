#ifndef SIMPLE_JOY_APP_APP_HPP
#define SIMPLE_JOY_APP_APP_HPP

#include <utility>

#include "packet_interfaces/msg/power.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace app {

class App : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;

    rclcpp::Publisher<packet_interfaces::msg::Power>::SharedPtr power_publisher;

    void joy_callback(const sensor_msgs::msg::Joy& msg);

    auto para_move_power(const std::pair<double, double>& stick
    ) -> packet_interfaces::msg::Power;

public:
    App(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // SIMPLE_JOY_APP_APP_HPP
