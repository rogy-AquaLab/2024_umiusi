#ifndef PACKET_QUIT_QUITSPACKET_HPP
#define PACKET_QUIT_QUITSPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Quit : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    Quit();
};

#endif // PACKET_QUIT_QUITSPACKET_HPP

