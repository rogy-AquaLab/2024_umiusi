#ifndef POWER_MAP_HPP
#define POWER_MAP_HPP

#include <array>
#include <memory>
#include <utility>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "packet_interfaces/msg/power.hpp"
#include "power_map_msg/msg/normalized_power.hpp"

#include "power_map/container.hpp"
#include "power_map/placement.hpp"

namespace power_map {

class PowerMap : public rclcpp::Node {
private:
    using Config    = Container<int>;
    using CbHandles = Container<std::shared_ptr<rclcpp::ParameterCallbackHandle>>;

    std::shared_ptr<rclcpp::ParameterEventHandler> param_cb;

    std::array<Config, 4>    configs;
    std::array<CbHandles, 4> cb_handles;

    std::array<Placement, 4> bldc_placement_config;
    std::array<Placement, 4> servo_placement_config;

    std::shared_ptr<rclcpp::ParameterCallbackHandle> bldc_placement_cb_handle;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> servo_placement_cb_handle;

    rclcpp::Publisher<packet_interfaces::msg::Power>::SharedPtr publisher;

    rclcpp::Subscription<power_map_msg::msg::NormalizedPower>::SharedPtr subscription;

    auto create_bldc_center_cb(size_t i
    ) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType;

    auto create_bldc_positive_radius_cb(size_t i
    ) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType;

    auto create_bldc_negative_radius_cb(size_t i
    ) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType;

    auto create_servo_min_cb(size_t i
    ) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType;

    auto create_servo_max_cb(size_t i
    ) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType;

    auto bldc_placement_param_cb(const rclcpp::Parameter& param) -> void;

    auto servo_placement_param_cb(const rclcpp::Parameter& param) -> void;

    auto subscription_callback(const power_map_msg::msg::NormalizedPower& msg) -> void;

public:
    PowerMap(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
};

}

#endif // POWER_MAP_HPP
