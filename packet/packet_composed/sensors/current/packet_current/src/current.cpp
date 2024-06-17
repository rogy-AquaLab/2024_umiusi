#include <chrono>
#include <functional>
#include <string>

#include "packet_current/currentspacket.hpp"

void Current::_loop() {
    std::stringstream ss;
    ss << "Hello, world! " << _count;
    std_msgs::msg::String msg{};
    msg.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "say %s", ss.str().c_str());
    _publisher->publish(msg);
    _count++;
}


Current::Current() :
    rclcpp::Node("Current"),
    _publisher(),
    _timer(),  
    _count(0)  
{
    using namespace std::chrono_literals;
    _publisher = this->create_publisher<std_msgs::msg::String>("/currents_composed", 10);
    auto loop = std::bind(&Current::_loop, this);
    _timer = this->create_wall_timer(500ms, loop);
}
