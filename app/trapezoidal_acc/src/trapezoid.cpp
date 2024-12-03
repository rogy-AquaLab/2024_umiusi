#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "trapezoidal_acc/trapezoid.hpp"

Trapezoid::Trapezoid(): 
    rclcpp::Node("trapezoid_acc"),
    current_velocity(0.0),
    target_velocity(10.0)
{
    trapezoid_publisher = this->create_publisher<power_map_msg::msg::NormalizedPower>("/app/unsafe_power", 10);
    trapezoid_subscription = this->create_subscription<power_map_msg::msg::NormalizedPower>(
        "safe_power", 10, std::bind(&Trapezoid::trapezoid_topic_callback, this, std::placeholders::_1));

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Trapezoid::_loop, this));
    
    std::fill(previous_bldc.begin(), previous_bldc.end(), 0.0);
    std::fill(previous_servo.begin(), previous_servo.end(), 0.0);
}

void Trapezoid::trapezoid_topic_callback(const power_map_msg::msg::NormalizedPower& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received trapezoid_message");
    normalized_power = msg;
}

void Trapezoid::_loop()
{
    double threshold_velocity = 5.0;
    double delta_limit = 0.05;

    if (current_velocity > threshold_velocity) 
    {
        current_velocity = threshold_velocity;
    }

    power_map_msg::msg::NormalizedPower msg;
    for (int i = 0; i < 4; i++) 
    {
        double target_bldc = normalized_power.bldc[i] * (current_velocity / target_velocity);
        double bldc_change = target_bldc - previous_bldc[i];

        if (std::abs(bldc_change) > delta_limit) 
        {
            target_bldc = previous_bldc[i] + std::copysign(delta_limit, bldc_change);
        }

        msg.bldc[i] = target_bldc;
        previous_bldc[i] = target_bldc;

        double target_servo = (current_velocity > 0.0) ? 1.0 : -1.0;
        double servo_change = target_servo - previous_servo[i];

        if (std::abs(servo_change) > delta_limit) {
            target_servo = previous_servo[i] + std::copysign(delta_limit, servo_change);
        }

        msg.servo[i] = target_servo;
        previous_servo[i] = target_servo;
    }

    trapezoid_publisher->publish(msg);
}
