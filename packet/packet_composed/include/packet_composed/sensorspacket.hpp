#ifndef PACKET_COMPOSED_SENSORSPACKET_HPP
#define PACKET_COMPOSED_SENSORSPACKET_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "packet_interfaces/msg/composed.hpp"
// #include "packet_interfaces/msg/depth.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "packet_interfaces/msg/flex.hpp"
#include "packet_interfaces/msg/current.hpp"
#include "packet_interfaces/msg/voltage.hpp"

class Composed : public rclcpp::Node {
private:
    rclcpp::Publisher<packet_interfaces::msg::Composed>::SharedPtr composed_publisher;
    // rclcpp::Subscription<packet_interfaces::msg::Depth>::SharedPtr depth_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Flex>::SharedPtr  flex1_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Flex>::SharedPtr  flex2_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Current>::SharedPtr  current_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Voltage>::SharedPtr  voltage_subscription;

    rclcpp::TimerBase::SharedPtr _timer;
    packet_interfaces::msg::Composed composed_msg;

    void _loop();
    // void depth_topic_callback(const packet_interfaces::msg::Depth& msg);
    void imu_topic_callback(const sensor_msgs::msg::Imu& msg);
    void flex1_topic_callback(const packet_interfaces::msg::Flex& msg);
    void flex2_topic_callback(const packet_interfaces::msg::Flex& msg);
    void current_topic_callback(const packet_interfaces::msg::Current& msg);
    void voltage_topic_callback(const packet_interfaces::msg::Voltage& msg);

    // packet_interfaces::msg::Depth _depth;
    sensor_msgs::msg::Imu _imu;
    packet_interfaces::msg::Flex _flex1;
    packet_interfaces::msg::Flex _flex2;
    packet_interfaces::msg::Current _current;
    packet_interfaces::msg::Voltage _voltage;

    // bool _depth_received;
    bool _imu_received;
    bool _flex1_received;
    bool _flex2_received;
    bool _current_received;
    bool _voltage_received;

public:
    Composed();
};

#endif // PACKET_COMPOSED_SENSORPACKET_HPP
