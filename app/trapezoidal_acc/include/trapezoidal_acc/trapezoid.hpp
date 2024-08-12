#ifndef TRAPEZOIDAL_ACC_TRAPEZOID_HPP
#define TRAPEZOIDAL_ACC_TRAPEZOID_HPP

#include <memory>
#include <array>
#include <power_map_msg/msg/detail/normalized_power__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "power_map_msg/msg/normalized_power.hpp"

class Trapezoid : public rclcpp::Node {

public:
    Trapezoid();

private:
    rclcpp::Publisher<power_map_msg::msg::NormalizedPower>::SharedPtr trapezoid_publisher;
    rclcpp::Subscription<power_map_msg::msg::NormalizedPower>::SharedPtr trapezoid_subscription;

    rclcpp::TimerBase::SharedPtr timer;
    power_map_msg::msg::NormalizedPower normalizedpower_msg;

    void _loop();
    void trapezoid_topic_callback(const power_map_msg::msg::NormalizedPower& msg);

    double current_velocity;
    double target_velocity;

    power_map_msg::msg::NormalizedPower normalized_power;

    std::array<double, 4> previous_bldc;
    std::array<double, 4> previous_servo; 

};

#endif // TRAPEZOIDAL_ACC_TRAPEZOID_HPP
