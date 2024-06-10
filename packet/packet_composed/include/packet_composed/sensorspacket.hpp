#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#ifndef PACKET_COMPOSED_SENSORPACKET_HPP
#define PACKET_COMPOSED_SENSORPACKET_HPP

class composed : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    composed();
};

#endif // PACKET_COMPOSED_SENSORPACKET_HPP