#ifndef PACKET_CURRENT_CURRENTSPACKET_HPP
#define PACKET_CURRENT_CURRENTSPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Current : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    Current();
};

#endif // PACKET_CURRENT_CURRENTSPACKET_HPP
