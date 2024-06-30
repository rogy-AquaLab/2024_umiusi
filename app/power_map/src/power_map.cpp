#include "power_map/power_map.hpp"

#include <cstdint>
#include <string>
#include <unordered_set>

#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <packet_interfaces/msg/power.hpp>
#include <power_map_msg/msg/normalized_power.hpp>

// FIXME: create_***_cbのコピペを辞めたい
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

    std::array<size_t, 4> bldc_placement_map{}, servo_placement_map{};
    for (size_t i = 0; i < 4; ++i) {
        bldc_placement_map[static_cast<size_t>(this->bldc_placement_config[i])]   = i;
        servo_placement_map[static_cast<size_t>(this->servo_placement_config[i])] = i;
    }

    packet_interfaces::msg::Power pub_msg{};
    for (size_t i = 0; i < 4; ++i) {
        const size_t bldc_index  = bldc_placement_map[i];
        const size_t servo_index = servo_placement_map[i];
        // FIXME: formatが
        const auto& bldc_config  = this->configs[bldc_index];
        const auto& servo_config = this->configs[servo_index];
        const float bldc_radius  = msg.bldc[i] < 0 ? bldc_config.bldc_negative_radius()
                                                   : bldc_config.bldc_positive_radius();
        pub_msg.bldc[bldc_index] = static_cast<std::uint16_t>(
            bldc_config.bldc_center() + static_cast<int>(msg.bldc[i] * bldc_radius)
        );
        const float servo_range
            = static_cast<float>(servo_config.servo_max() - servo_config.servo_min());
        pub_msg.servo[servo_index] = static_cast<std::uint16_t>(
            servo_config.servo_min()
            + static_cast<int>((msg.servo[i] + 1) / 2.0 * servo_range)
        );
    }
    this->publisher->publish(pub_msg);
}

auto power_map::PowerMap::bldc_placement_param_cb(const rclcpp::Parameter& param
) -> void {
    RCLCPP_INFO(this->get_logger(), "received parameter update of bldc_placement");
    const auto& placement_str = param.as_string_array();
    for (size_t i = 0; i < 4; ++i) {
        this->bldc_placement_config[i] = placement_from_str(placement_str[i]).value();
    }
}

auto power_map::PowerMap::servo_placement_param_cb(const rclcpp::Parameter& param
) -> void {
    RCLCPP_INFO(this->get_logger(), "received parameter update of servo_placement");
    const auto& placement_str = param.as_string_array();
    for (size_t i = 0; i < 4; ++i) {
        this->servo_placement_config[i] = placement_from_str(placement_str[i]).value();
    }
}

auto power_map::PowerMap::on_set_parameters_cb(
    const std::vector<rclcpp::Parameter>& params
) -> rcl_interfaces::msg::SetParametersResult {
    using rclcpp::ParameterType;

#define RETURN_RESULT(SUCCESSFUL, REASON)                \
    do {                                                 \
        rcl_interfaces::msg::SetParametersResult result; \
        result.successful = SUCCESSFUL;                  \
        result.reason     = REASON;                      \
        return result;                                   \
    } while (false)

    for (const rclcpp::Parameter& param : params) {
        const std::string&   name = param.get_name();
        const ParameterType& type = param.get_type();
        if (name.find("bldc_center") == 0 || name.find("bldc_positive_radius") == 0
            || name.find("bldc_negative_radius") == 0 || name.find("servo_min") == 0
            || name.find("servo_max") == 0)
        {
            if (type != ParameterType::PARAMETER_INTEGER) {
                RETURN_RESULT(false, "invalid type; expected integer");
            }
        } else if (name == "servo_placement" || name == "bldc_placement") {
            if (type != ParameterType::PARAMETER_STRING_ARRAY) {
                RETURN_RESULT(false, "invalid type; expected array of string");
            }
            const std::vector<std::string>& value = param.as_string_array();
            if (value.size() != 4) {
                RETURN_RESULT(false, "invalid value; the length of array must be 4");
            }
            std::unordered_set<power_map::Placement> seen_placements;
            for (const std::string& v : value) {
                const std::optional<power_map::Placement> parsed = placement_from_str(v);
                if (!parsed.has_value()) {
                    RETURN_RESULT(
                        false,
                        "invalid value; each element of array must be one of left-front, "
                        "left-back, right-front, right-back"
                    );
                }
                const power_map::Placement placement = parsed.value();
                if (seen_placements.count(placement) > 0) {
                    RETURN_RESULT(false, "invalid value; each element must be unique");
                }
                seen_placements.insert(placement);
            }
        } else {
            RCLCPP_DEBUG_STREAM(
                this->get_logger(), "received unexpected parameter " << name << ", skip"
            );
        }
    }
    RETURN_RESULT(true, "");

#undef RETURN_RESULT
}

power_map::PowerMap::PowerMap(const rclcpp::NodeOptions& options) :
    rclcpp::Node("power_map", options) {
    constexpr int DEFAULT_BLDC_CENTER          = 1480;
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
    {
        // bldc placement
        const std::string parameter_name = "bldc_placement";
        this->declare_parameter(
            parameter_name,
            std::vector<std::string>{
                "left-front", "right-front", "left-back", "right-back" }
        );
        this->bldc_placement_param_cb(this->get_parameter(parameter_name));
        this->bldc_placement_cb_handle = this->param_cb->add_parameter_callback(
            parameter_name,
            [this](const rclcpp::Parameter& param) {
                this->bldc_placement_param_cb(param);
            }
        );
    }
    {
        // servo placement
        const std::string parameter_name = "servo_placement";
        this->declare_parameter(
            parameter_name,
            std::vector<std::string>{
                "left-front", "right-front", "left-back", "right-back" }
        );
        this->servo_placement_param_cb(this->get_parameter(parameter_name));
        this->servo_placement_cb_handle = this->param_cb->add_parameter_callback(
            parameter_name,
            [this](const rclcpp::Parameter& param) {
                this->servo_placement_param_cb(param);
            }
        );
    }
    this->on_set_parameters_cb_handle = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            return this->on_set_parameters_cb(params);
        }
    );

    this->publisher = this->create_publisher<packet_interfaces::msg::Power>("power", 10);
    this->subscription = this->create_subscription<power_map_msg::msg::NormalizedPower>(
        "normalized_power",
        10,
        [this](const power_map_msg::msg::NormalizedPower& msg) {
            this->subscription_callback(msg);
        }
    );
}

RCLCPP_COMPONENTS_REGISTER_NODE(power_map::PowerMap)
