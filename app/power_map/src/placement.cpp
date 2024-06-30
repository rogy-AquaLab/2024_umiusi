#include "power_map/placement.hpp"

auto power_map::placement_from_str(const std::string& value) -> std::optional<Placement> {
    if (value == "left-front") {
        return Placement::LeftFront;
    }
    if (value == "right-front") {
        return Placement::RightFront;
    }
    if (value == "left-back") {
        return Placement::LeftBack;
    }
    if (value == "right-back") {
        return Placement::RightBack;
    }
    return std::nullopt;
}
