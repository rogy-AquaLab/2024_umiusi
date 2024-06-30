#include <chrono>
#include <functional>
#include <string>
#include "packet_composed/sensorspacket.hpp"


Composed::Composed() :
    rclcpp::Node("composed"),
    _timer(),
    _depth(),
    _imu(),
    _flex1(),
    _flex2(),
    _current(),
    _voltage(),
    _depth_received(false),
    _imu_received(false),
    _flex1_received(false),
    _flex2_received(false),
    _current_received(false),
    _voltage_received(false)
    _count(0)  
{
    //publisher,subscriptionを作る。

    using namespace std::chrono_literals;
    composed_publisher = this->create_publisher<packet_interfaces::msg::Composed>("sensors_composed", 10);

    depth_subscription = this->create_subscription<packet_interfaces::msg::Depth>(
        "sensors/depth", 10, std::bind(&Composed::depth_topic_callback, this, std::placeholders::_1));

    imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
        "sensors/imu", 10, std::bind(&Composed::imu_topic_callback, this, std::placeholders::_1));

    flex1_subscription = this->create_subscription<packet_interfaces::msg::Flex>(
        "sensors/flex/1", 10, std::bind(&Composed::flex1_topic_callback, this, std::placeholders::_1));

    flex2_subscription = this->create_subscription<packet_interfaces::msg::Flex>(
        "sensors/flex/2", 10, std::bind(&Composed::flex2_topic_callback, this, std::placeholders::_1));
    
    current_subscription = this->create_subscription<packet_interfaces::msg::Current>(
        "sensors/current", 10, std::bind(&Composed::current_topic_callback, this, std::placeholders::_1));

    voltage_subscription = this->create_subscription<packet_interfaces::msg::Voltage>(
        "sensors/voltage", 10, std::bind(&Composed::, this, std::placeholders::_1));
   
    auto loop = std::bind(&Composed::_loop, this);
    _timer = this->create_wall_timer(500ms, loop);
}

void Composed::_loop() {

    //下のif文のcallbackとtimerのcallbackは非同期に呼ばれるからできなさそう。。。↓
    if (_depth_received && _imu_received && _flex1_received && _flex2_received && _current_received && _voltage_received) 
    {
        auto composed_msg = packet_interfaces::msg::Composed();
        composed_msg.depth = _depth;
        composed_msg.imu = _imu;
        composed_msg.flex1 = _flex1;
        composed_msg.flex2 = _flex2;
        composed_msg.current = _current;
        composed_msg.voltage = _voltage;
        
        composed_publisher->publish(composed_msg); 

        // メッセージを再度受信させる。
        _depth_received = false;
        _imu_received = false;
        _flex1_received = false;
        _flex2_received = false;
        _current_received = false;
        _voltage_received = false;

    //文字列はもういらない？？
    // std::stringstream ss;
    // ss << "Hello, world! " << _count;
    // std_msgs::msg::String msg{};
    // msg.data = ss.str();
    // RCLCPP_INFO(this->get_logger(), "say %s", ss.str().c_str());
    // composed_publisher->publish(composed_msg);
    // _count++;
    }
}

void Composed::depth_topic_callback(const packet_interfaces::msg::Depth& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Depth message");
    // Composedメッセージにデータを追加
    auto composed_msg = packet_interfaces::msg::Composed();
    // composed_msg.depth = msg;

    _depth = msg;  // 受信した深度データを保存   
    _depth_received = true;

    // 他のデータも追加する
    // composed_publisher->publish(composed_msg); ここでやらず_loopでする？
}

void Composed::imu_topic_callback(const sensor_msgs::msg::Imu& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received IMU message");
    // Composedメッセージにデータを追加
    // auto composed_msg = packet_interfaces::msg::Composed();
    // composed_msg.imu = msg;

    _imu = msg;  // 受信したIMUデータを保存
    _imu_received = true;

    // 他のデータも追加する
    // composed_publisher->publish(composed_msg); ここでやらず_loopでする？
}

void Composed::flex1_topic_callback(const packet_interfaces::msg::Flex1& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Flex1 message");
    // Composedメッセージにデータを追加
    // auto composed_msg = packet_interfaces::msg::Composed();
    // composed_msg.flex1 = msg;

    _flex1 = msg;  // 受信したFlex1データを保存
    _flex1_received = true; 

    //composed_publisher->publish(composed_msg); ここでやらず_loopでする？
}

void Composed::flex2_topic_callback(const packet_interfaces::msg::Flex2& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Flex2 message");
    // Composedメッセージにデータを追加
    // auto composed_msg = packet_interfaces::msg::Composed();
    // composed_msg.flex2 = msg;

    _flex2 = msg;  // 受信したFlex2データを保存
    _flex2_received = true;

    //composed_publisher->publish(composed_msg); ここでやらず_loopでする？
}

void Composed::current_topic_callback(const packet_interfaces::msg::Current2& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Current message");
    // Composedメッセージにデータを追加
    // auto composed_msg = packet_interfaces::msg::Composed();
    // composed_msg.Current = msg;

    _current = msg;  // 受信したCurrentデータを保存
    _current_received = true;

    //composed_publisher->publish(composed_msg); ここでやらず_loopでする？
}

void Composed::voltage_topic_callback(const packet_interfaces::msg::Voltage& msg)
{
    RCLCPP_DEBUG(this->get_logger(), "Received Flex2 message");
    // Composedメッセージにデータを追加
    // auto composed_msg = packet_interfaces::msg::Composed();
    // composed_msg.voltage = msg;

    _voltage = msg;  // 受信したVoltageデータを保存
    _voltage_received = true;

    //composed_publisher->publish(composed_msg); ここでやらず_loopでする？
}

