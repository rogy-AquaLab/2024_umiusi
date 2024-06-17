#ifndef PACKET_FLEX2_FLEX1SPACKET_HPP
#define PACKET_FLEX2_FLEX1SPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Flex2 : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    Flex2();
};

#endif // PACKET_FLEX2_FLEX1SPACKET_HPP
