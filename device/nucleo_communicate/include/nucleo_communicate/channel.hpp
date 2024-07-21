#ifndef NUCLEO_COMMUNICATE_CHANNEL_HPP
#define NUCLEO_COMMUNICATE_CHANNEL_HPP

#include <cstdio>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

namespace channel {

class Channel : public rclcpp::Node {
private:
    FILE* serial_port;

public:
    Channel(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // NUCLEO_COMMUNICATE_CHANNEL_HPP
