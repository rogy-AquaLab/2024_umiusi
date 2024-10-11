#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <rclcpp/logging.hpp>
#include <utility>

#include <rclcpp/time.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "nucleo_communicate/channel.hpp"
#include "nucleo_communicate/recv_data.hpp"
#include "nucleo_communicate/send_data.hpp"

using packet_interfaces::msg::Current;
using packet_interfaces::msg::Flex;
using packet_interfaces::msg::NucleoState;
using packet_interfaces::msg::Power;
using packet_interfaces::msg::Voltage;

void channel::Channel::initialize_subscription_callback(const std_msgs::msg::Empty&) {
    const std::lock_guard<std::mutex> _guard(this->serial_mutex);
    const bool                        success = this->serial.initialize();
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Sent initialize order");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send initialize order");
    }
}

void channel::Channel::suspend_subscription_callback(const std_msgs::msg::Empty&) {
    const std::lock_guard<std::mutex> _guard(this->serial_mutex);
    const bool                        success = this->serial.suspend();
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Sent suspend order");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send suspend order");
    }
}

/* タイマーとパワーsubscriptionは頻繁に呼ばれることを想定してnon-blockingに */

void channel::Channel::power_subscription_callback(const Power& msg) {
    const std::unique_lock<std::mutex> lock(this->serial_mutex, std::try_to_lock);

    if (!lock.owns_lock()) {
        return;
    }

    const nucleo_com::SendData data    = nucleo_com::SendData::from_msg(msg);
    const bool                 success = this->serial.send(data);
    if (success) {
        RCLCPP_INFO(this->get_logger(), "Sent power order");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to send power order");
    }
}

void channel::Channel::timer_callback() {
    const std::unique_lock<std::mutex> lock(this->serial_mutex, std::try_to_lock);

    if (!lock.owns_lock()) {
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "tick");
    const rclcpp::Time now = this->get_clock()->now();

    const std::optional<nucleo_com::NucleoState> state_data
        = this->serial.receive_state();
    if (state_data.has_value()) {
        NucleoState state_msg;
        state_msg.header.frame_id = "nucleo_state";
        state_msg.header.stamp    = now;
        state_msg.state           = static_cast<std::uint8_t>(state_data.value());
        this->state_publisher->publish(state_msg);
    }
    const std::optional<nucleo_com::RecvData> data = this->serial.receive();
    if (!data.has_value()) {
        return;
    }
    {
        Flex flex1_msg;
        flex1_msg.header.frame_id = "nucleo_flex_1";
        flex1_msg.header.stamp    = now;
        flex1_msg.value           = data->flex1_value();
        this->flex1_publisher->publish(std::move(flex1_msg));
    }
    {
        Flex flex2_msg;
        flex2_msg.header.frame_id = "nucleo_flex_2";
        flex2_msg.header.stamp    = now;
        flex2_msg.value           = data->flex2_value();
        this->flex2_publisher->publish(std::move(flex2_msg));
    }
    {
        Current current_msg;
        current_msg.header.frame_id = "nucleo_current";
        current_msg.header.stamp    = now;
        current_msg.value           = data->current_value();
        this->current_publisher->publish(std::move(current_msg));
    }
    {
        Voltage voltage_msg;
        voltage_msg.header.frame_id = "nucleo_voltage";
        voltage_msg.header.stamp    = now;
        voltage_msg.value           = data->voltage_value();
        this->voltage_publisher->publish(std::move(voltage_msg));
    }
}

channel::Channel::Channel(const rclcpp::NodeOptions& options) :
    rclcpp::Node("channel", options),
    serial("/dev/ttyACM0"),
    serial_mutex(),
    order_failed_at(std::nullopt),
    initialize_subscription(nullptr),
    suspend_subscription(nullptr),
    power_subscription(nullptr),
    state_publisher(nullptr),
    flex1_publisher(nullptr),
    flex2_publisher(nullptr),
    current_publisher(nullptr),
    voltage_publisher(nullptr),
    timer(nullptr) {
    using namespace std::chrono_literals;
    using std::placeholders::_1;

    this->serial.setup();

    this->initialize_subscription = this->create_subscription<std_msgs::msg::Empty>(
        "initialize",
        10,
        std::bind(&channel::Channel::initialize_subscription_callback, this, _1)
    );
    this->suspend_subscription = this->create_subscription<std_msgs::msg::Empty>(
        "suspend",
        10,
        std::bind(&channel::Channel::suspend_subscription_callback, this, _1)
    );
    this->power_subscription = this->create_subscription<Power>(
        "power", 10, std::bind(&channel::Channel::power_subscription_callback, this, _1)
    );

    this->state_publisher   = this->create_publisher<NucleoState>("nucleo_state", 10);
    this->flex1_publisher   = this->create_publisher<Flex>("flex_1", 10);
    this->flex2_publisher   = this->create_publisher<Flex>("flex_2", 10);
    this->current_publisher = this->create_publisher<Current>("current", 10);
    this->voltage_publisher = this->create_publisher<Voltage>("voltage", 10);

    this->timer = this->create_wall_timer(
        70ms, std::bind(&channel::Channel::timer_callback, this)
    );
}

RCLCPP_COMPONENTS_REGISTER_NODE(channel::Channel)
