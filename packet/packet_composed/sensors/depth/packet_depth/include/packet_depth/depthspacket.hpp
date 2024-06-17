#ifndef PACKET_DEPTH_DEPTHSPACKET_HPP
#define PACKET_DEPTH_DEPTHSPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Depth : public rclcpp::Node {
private:

    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> _publisher;
    std::shared_ptr<rclcpp::TimerBase> _timer;
    unsigned int _count;
    void _loop();

public:
    Depth();
};

#endif // PACKET_DEPTH_DEPTHSPACKET_HPP
