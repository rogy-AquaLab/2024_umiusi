#ifndef PACKET_COMPOSED_SENSORSPACKET_HPP
#define PACKET_COMPOSED_SENSORSPACKET_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Composed : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _publisher;
    rclcpp::Publisher<packet_interfaces::msg::Composed>::SharedPtr composed_publisher;

    rclcpp::Subscription<packet_interfaces::msg::Depth>::SharedPtr depth_subscription;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Flex>::SharedPtr  flex1_subscription;
    rclcpp::Subscription<packet_interfaces::msg::Flex>::SharedPtr  flex2_subscription;
    rclcpp::Subscription<packet_interfaces::msg::current>::SharedPtr  current_subscription;
    rclcpp::Subscription<packet_interfaces::msg::voltage>::SharedPtr  voltage_subscription;

    //まとめてpublishするためにタイマーのつもり
    rclcpp::TimerBase::SharedPtr _timer;
    size_t _count;
    packet_interfaces::msg::Composed composed_msg;

    void _loop();
    void depth_topic_callback(const packet_interfaces::msg::Depth& msg);
    void imu_topic_callback(const sensor_msgs::msg::Imu& msg);
    void flex1_topic_callback(const packet_interfaces::msg::Flex1& msg);
    void flex2_topic_callback(const packet_interfaces::msg::Flex2& msg);
    void current_topic_callback(const packet_interfaces::msg::current& msg);
    void voltage_topic_callback(const packet_interfaces::msg::voltage& msg);
 

    // さらにいくつかのメソッド, プロパティが必要になる

public:
    Composed();
};

#endif // PACKET_COMPOSED_SENSORPACKET_HPP
