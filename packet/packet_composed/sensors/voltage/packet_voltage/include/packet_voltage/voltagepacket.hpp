#ifndef PACKET_VOLTAGE_VOLTAGEPACKET_HPP
#define PACKET_VOLTAGE_VOLTAGEPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Voltage : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    Voltage();
};

#endif // PACKET_VOLTAGE_VOLTAGEPACKET_HPP
