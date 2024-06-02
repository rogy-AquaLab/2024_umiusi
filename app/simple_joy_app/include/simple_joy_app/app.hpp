#ifndef SIMPLE_JOY_APP_APP_HPP
#define SIMPLE_JOY_APP_APP_HPP

#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace app {

class App : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription;

    void joy_callback(const sensor_msgs::msg::Joy& msg);

public:
    App(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // SIMPLE_JOY_APP_APP_HPP
