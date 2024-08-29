#ifndef NUCLEO_COMMUNICATE_CHANNEL_HPP
#define NUCLEO_COMMUNICATE_CHANNEL_HPP

#include <mutex>
#include <optional>

#include <packet_interfaces/msg/current.hpp>
#include <packet_interfaces/msg/flex.hpp>
#include <packet_interfaces/msg/nucleo_state.hpp>
#include <packet_interfaces/msg/power.hpp>
#include <packet_interfaces/msg/voltage.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/empty.hpp>

#include "nucleo_communicate/serial_port.hpp"

namespace channel {

class Channel : public rclcpp::Node {
private:
    nucleo_com::SerialPort serial;
    std::mutex             serial_mutex;

    std::optional<rclcpp::Time> order_failed_at;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr initialize_subscription;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr suspend_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Power>::SharedPtr power_subscription;

    rclcpp::Publisher<packet_interfaces::msg::NucleoState>::SharedPtr state_publisher;
    rclcpp::Publisher<packet_interfaces::msg::Flex>::SharedPtr        flex1_publisher;
    rclcpp::Publisher<packet_interfaces::msg::Flex>::SharedPtr        flex2_publisher;
    rclcpp::Publisher<packet_interfaces::msg::Current>::SharedPtr     current_publisher;
    rclcpp::Publisher<packet_interfaces::msg::Voltage>::SharedPtr     voltage_publisher;

    rclcpp::TimerBase::SharedPtr timer;

    void initialize_subscription_callback(const std_msgs::msg::Empty& _msg);
    void suspend_subscription_callback(const std_msgs::msg::Empty& _msg);
    void power_subscription_callback(const packet_interfaces::msg::Power& msg);

    void timer_callback();

public:
    Channel(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // NUCLEO_COMMUNICATE_CHANNEL_HPP
