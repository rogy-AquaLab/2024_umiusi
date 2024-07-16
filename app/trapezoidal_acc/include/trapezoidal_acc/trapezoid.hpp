#ifndef TRAPEZOIDAL_ACC_TRAPEZOID_HPP
#define TRAPEZOIDAL_ACC_TRAPEZOID_HPP

#include <memory>
#include <power_map_msg/msg/detail/normalized_power__struct.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "power_map_msg/msg/normalized_power.hpp"

class Trapezoid : public rclcpp::Node {

public:
    Trapezoid();

private:
    rclcpp::Publisher<power_map_msg::msg::NormalizedPower>::SharedPtr Trapezoid_publisher;
    rclcpp::Subscription<power_map_msg::msg::NormalizedPower>::SharedPtr Trapezoid_subscription;

    rclcpp::TimerBase::SharedPtr _timer;
    power_map_msg::msg::NormalizedPower NormalizedPower_msg;

    void _loop();
    void trapezoid_topic_callback(const power_map_msg::msg::NormalizedPower& msg);

    double _current_velocity;
    double _target_velocity;

    power_map_msg::msg::NormalizedPower _NormalizedPower;

};

#endif // TRAPEZOIDAL_ACC_TRAPEZOID_HPP
