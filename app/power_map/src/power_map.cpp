#include "power_map/power_map.hpp"
#include <cstdint>
#include <packet_interfaces/msg/detail/power__struct.hpp>
#include <power_map_msg/msg/detail/normalized_power__struct.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <string>

auto power_map::PowerMap::create_bldc_center_cb(size_t i
) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType {
    return [this, i](const rclcpp::Parameter& p) {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "received parameter update of " << "bldc_center" << (i + 1)
        );
        this->configs[i].bldc_center(p.as_int());
    };
}

auto power_map::PowerMap::create_bldc_positive_radius_cb(size_t i
) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType {
    return [this, i](const rclcpp::Parameter& p) {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "received parameter update of " << "bldc_positive_radius" << (i + 1)
        );
        this->configs[i].bldc_positive_radius(p.as_int());
    };
}

auto power_map::PowerMap::create_bldc_negative_radius_cb(size_t i
) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType {
    return [this, i](const rclcpp::Parameter& p) {
        RCLCPP_INFO_STREAM(
            this->get_logger(),
            "received parameter update of " << "bldc_negative_radius" << (i + 1)
        );
        this->configs[i].bldc_negative_radius(p.as_int());
    };
}

auto power_map::PowerMap::create_servo_min_cb(size_t i
) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType {
    return [this, i](const rclcpp::Parameter& p) {
        RCLCPP_INFO_STREAM(
            this->get_logger(), "received parameter update of " << "servo_min" << (i + 1)
        );
        this->configs[i].servo_min(p.as_int());
    };
}

auto power_map::PowerMap::create_servo_max_cb(size_t i
) -> rclcpp::ParameterCallbackHandle::ParameterCallbackType {
    return [this, i](const rclcpp::Parameter& p) {
        RCLCPP_INFO_STREAM(
            this->get_logger(), "received parameter update of " << "servo_max" << (i + 1)
        );
        this->configs[i].servo_max(p.as_int());
    };
}

auto power_map::PowerMap::subscription_callback(
    const power_map_msg::msg::NormalizedPower& msg
) -> void {
    RCLCPP_DEBUG(this->get_logger(), "received normalized power");
    packet_interfaces::msg::Power pub_msg{};
    for (size_t i = 0; i < 4; ++i) {
        // FIXME: formatが
        const auto& config      = this->configs[i];
        const float bldc_radius = msg.bldc[i] < 0 ? config.bldc_negative_radius()
                                                  : config.bldc_positive_radius();
        pub_msg.bldc[i]         = static_cast<std::uint16_t>(
            config.bldc_center() + static_cast<int>(msg.bldc[i] * bldc_radius)
        );
        const float servo_range
            = static_cast<float>(config.servo_max() - config.servo_min());
        pub_msg.servo[i] = static_cast<std::uint16_t>(
            config.servo_min() + static_cast<int>((msg.servo[i] + 1) / 2.0 * servo_range)
        );
    }
    this->publisher->publish(pub_msg);
}

power_map::PowerMap::PowerMap(const rclcpp::NodeOptions& options) :
    rclcpp::Node("power_map", options) {
    constexpr int DEFAULT_BLDC_CENTER          = 1048;
    constexpr int DEFAULT_BLDC_POSITIVE_RADIUS = 250;
    constexpr int DEFAULT_BLDC_NEGATIVE_RADIUS = 250;
    constexpr int DEFAULT_SERVO_MIN            = 500;
    constexpr int DEFAULT_SERVO_MAX            = 2400;

    this->param_cb = std::make_shared<rclcpp::ParameterEventHandler>(this);
    for (size_t i = 0; i < 4; ++i) {
        {
            // bldc_center
            const auto parameter_name = "bldc_center" + std::to_string(i + 1);
            this->declare_parameter(parameter_name, DEFAULT_BLDC_CENTER);
            this->configs[i].bldc_center(this->get_parameter(parameter_name).as_int());
            auto cb = this->param_cb->add_parameter_callback(
                parameter_name, this->create_bldc_center_cb(i)
            );
            this->cb_handles[i].bldc_center(std::move(cb));
        }
        {
            // bldc_positive_radius
            const auto parameter_name = "bldc_positive_radius" + std::to_string(i + 1);
            this->declare_parameter(parameter_name, DEFAULT_BLDC_POSITIVE_RADIUS);
            this->configs[i].bldc_center(this->get_parameter(parameter_name).as_int());
            auto cb = this->param_cb->add_parameter_callback(
                parameter_name, this->create_bldc_positive_radius_cb(i)
            );
            this->cb_handles[i].bldc_positive_radius(std::move(cb));
        }
        {
            // bldc_negative_radius
            const auto parameter_name = "bldc_negative_radius" + std::to_string(i + 1);
            this->declare_parameter(parameter_name, DEFAULT_BLDC_NEGATIVE_RADIUS);
            this->configs[i].bldc_center(this->get_parameter(parameter_name).as_int());
            auto cb = this->param_cb->add_parameter_callback(
                parameter_name, this->create_bldc_negative_radius_cb(i)
            );
            this->cb_handles[i].bldc_negative_radius(std::move(cb));
        }
        {
            // servo_min
            const auto parameter_name = "servo_min" + std::to_string(i + 1);
            this->declare_parameter(parameter_name, DEFAULT_SERVO_MIN);
            this->configs[i].bldc_center(this->get_parameter(parameter_name).as_int());
            auto cb = this->param_cb->add_parameter_callback(
                parameter_name, this->create_servo_min_cb(i)
            );
            this->cb_handles[i].servo_min(std::move(cb));
        }
        {
            // servo_max
            const auto parameter_name = "servo_max" + std::to_string(i + 1);
            this->declare_parameter(parameter_name, DEFAULT_SERVO_MAX);
            this->configs[i].bldc_center(this->get_parameter(parameter_name).as_int());
            auto cb = this->param_cb->add_parameter_callback(
                parameter_name, this->create_servo_max_cb(i)
            );
            this->cb_handles[i].servo_max(std::move(cb));
        }
    }

    this->publisher = this->create_publisher<packet_interfaces::msg::Power>(
        "/packet/order/power", 10
    );
    this->subscription = this->create_subscription<power_map_msg::msg::NormalizedPower>(
        "normalized_power",
        10,
        [this](const power_map_msg::msg::NormalizedPower& msg) {
            this->subscription_callback(msg);
        }
    );
}
