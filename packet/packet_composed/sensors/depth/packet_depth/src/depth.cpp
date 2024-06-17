#include <chrono>
#include <functional>
#include <string>

#include "packet_depth/depthspacket.hpp"

void Depth::_loop() {
    std::stringstream ss;
    ss << "Hello, world! " << _count;
    std_msgs::msg::String msg{};
    msg.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "say %s", ss.str().c_str());
    _publisher->publish(msg);
    _count++;
}


Depth::Depth() :
    rclcpp::Node("depth"),
    _publisher(),
    _timer(),  
    _count(0)  
{
    using namespace std::chrono_literals;
    _publisher = this->create_publisher<std_msgs::msg::String>("/depths_composed", 10);
    auto loop = std::bind(&Depth::_loop, this);
    _timer = this->create_wall_timer(500ms, loop);
}
