#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "trapezoidal_acc/trapezoid.hpp"

Trapezoid::Trapezoid(): 
  rclcpp::Node("trapezoid_acc"),
  _current_velocity(0.0), //現在の速度（初期化）
  _target_velocity(10.0)// 目標速度
{
    Trapezoid_publisher = this->create_publisher<power_map_msg::msg::NormalizedPower>("/app/unsafe_power", 10);
    Trapezoid_subscription = this->create_subscription<power_map_msg::msg::NormalizedPower>(
        "/app/safe_power", 10, std::bind(&Trapezoid::trapezoid_topic_callback, this, std::placeholders::_1));

    _timer = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&Trapezoid::_loop, this));
}

void Trapezoid::trapezoid_topic_callback(const power_map_msg::msg::NormalizedPower& msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received trapezoid_message");
}

void Trapezoid::_loop() //速度のアルゴリズムがあまり理解できていないです。。
{
  double threshold_velocity = 5.0; // 速度の限度（仮に5.0とする。）

  // 速度がしきい値を超えた場合に速度を変更
  if (_current_velocity > threshold_velocity) {
    _current_velocity = threshold_velocity;
  } 

  auto message = power_map_msg::msg::NormalizedPower();
  Trapezoid_publisher->publish(message);
}
