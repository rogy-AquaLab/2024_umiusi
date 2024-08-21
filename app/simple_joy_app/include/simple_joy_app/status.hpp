#ifndef SIMPLE_JOY_APP_STATUS_HPP
#define SIMPLE_JOY_APP_STATUS_HPP

#include <cstdint>
#include <string>

namespace app {

enum class Status : std::uint8_t {
    Stopped,
    Moving,
    NoInput,
};

constexpr char STATUS_STOPPED_STR[]  = "Stopped";
constexpr char STATUS_MOVING_STR[]   = "Moving";
constexpr char STATUS_NO_INPUT_STR[] = "NoInput";

std::string status_str(const Status& status) {
    return status == Status::Stopped  ? STATUS_STOPPED_STR
           : status == Status::Moving ? STATUS_MOVING_STR
                                      : STATUS_NO_INPUT_STR;
}

enum class NucleoState : std::uint8_t {
    Initializing = 0,
    Suspend      = 1,
    Running      = 2,
};

}

#endif // SIMPLE_JOY_APP_STATUS_HPP
