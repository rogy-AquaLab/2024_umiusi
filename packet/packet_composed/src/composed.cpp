#include <chrono>
#include <functional>
#include <string>
#include "packet_composed/sensorspacket.hpp"


Composed::Composed() :
    rclcpp::Node("composed"),
    _publisher(),
    _timer(),  
    _count(0)  
{
    //composed_publisherをコンストラクタで初期化した。
    using namespace std::chrono_literals;
    composed_publisher = this->create_publisher<packet_interfaces::msg::Composed>("composed_topic", 10);

    depth_subscription = this->create_subscription<packet_interfaces::msg::Depth>(
        "sensors_depth", 10, std::bind(&Composed::depth_topic_callback, this, std::placeholders::_1));

    imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
        "sensors_imu", 10, std::bind(&Composed::imu_topic_callback, this, std::placeholders::_1));

    flex1_subscription = this->create_subscription<packet_interfaces::msg::Flex>(
        "sensors_flex1", 10, std::bind(&Composed::flex1_topic_callback, this, std::placeholders::_1));

    flex2_subscription = this->create_subscription<packet_interfaces::msg::Flex>(
        "sensors_flex2", 10, std::bind(&Composed::flex2_topic_callback, this, std::placeholders::_1));

    current_subscription = this->create_subscription<packet_interfaces::msg::Current>(
        "sensors_current", 10, std::bind(&Composed::current_topic_callback, this, std::placeholders::_1));

    voltage_subscription = this->create_subscription<packet_interfaces::msg::Voltage>(
        "sensors_voltage", 10, std::bind(&Composed::voltage_topic_callback, this, std::placeholders::_1));
   
    auto loop = std::bind(&Composed::_loop, this);
    _timer = this->create_wall_timer(500ms, loop);
}

void Composed::_loop() {
    std::stringstream ss;
    ss << "Hello, world! " << _count;
    std_msgs::msg::String msg{};
    msg.data = ss.str();
    RCLCPP_INFO(this->get_logger(), "say %s", ss.str().c_str());
    _publisher->publish(msg);
    _count++;
}

void Composed::depth_topic_callback(const packet_interfaces::msg::Depth& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Depth message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    composed_msg.depth = msg;
    // 他のデータも追加する
    composed_publisher->publish(composed_msg);
}

void Composed::imu_topic_callback(const sensor_msgs::msg::Imu& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received IMU message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    composed_msg.imu = msg;
    // 他のデータも追加する
    composed_publisher->publish(composed_msg);
}

void Composed::flex1_topic_callback(const packet_interfaces::msg::Flex1& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Flex1 message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    composed_msg.flex1 = msg;
    // 他のデータも追加する
    composed_publisher->publish(composed_msg);
}

void Composed::flex2_topic_callback(const packet_interfaces::msg::Flex2& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Flex2 message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    composed_msg.flex2 = msg;
    // 他のデータも追加する
    composed_publisher->publish(composed_msg);
}

void Composed::current_topic_callback(const packet_interfaces::msg::Current2& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Current message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    composed_msg.Current = msg;
    // 他のデータも追加する
    composed_publisher->publish(composed_msg);
}

void Composed::voltage_topic_callback(const packet_interfaces::msg::Voltage& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Flex2 message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    composed_msg.voltage = msg;
    // 他のデータも追加する
    composed_publisher->publish(composed_msg);
}

