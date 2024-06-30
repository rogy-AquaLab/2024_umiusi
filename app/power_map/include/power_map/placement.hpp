#ifndef POWER_MAP_PLACEMENT_HPP
#define POWER_MAP_PLACEMENT_HPP

#include <optional>
#include <string>

namespace power_map {

enum class Placement : char {
    LeftFront,
    RightFront,
    LeftBack,
    RightBack,
};

auto placement_from_str(const std::string& value) -> std::optional<Placement>;

}

#endif // POWER_MAP_PLACEMENT_HPP
