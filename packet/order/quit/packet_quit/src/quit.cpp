#include <chrono>
#include <functional>
#include <string>

#include "packet_quit/quitspacket.hpp"

void Quit::_loop() {
    std::stringstream ss;
    ss << "Hello, world! " << _count;
    std_msgs::msg::String msg{};
    msg.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "say %s", ss.str().c_str());
    _publisher->publish(msg);
    _count++;
}


Quit::Quit() :
    rclcpp::Node("quit"),
    _publisher(),
    _timer(),  
    _count(0)  
{
    using namespace std::chrono_literals;
    _publisher = this->create_publisher<std_msgs::msg::String>("/quitspacket", 10);
    auto loop = std::bind(&Quit::_loop, this);
    _timer = this->create_wall_timer(500ms, loop);
}

