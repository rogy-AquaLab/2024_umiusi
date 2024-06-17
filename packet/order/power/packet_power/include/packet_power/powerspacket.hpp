#ifndef PACKET_POWER_POWERSPACKET_HPP
#define PACKET_POWER_POWERSPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Power : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    Power();
};

#endif // PACKET_POWER_POWERSPACKET_HPP
