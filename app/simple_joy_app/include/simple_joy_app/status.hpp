#ifndef SIMPLE_JOY_APP_STATUS_HPP
#define SIMPLE_JOY_APP_STATUS_HPP

#include <cstdint>

namespace app {

enum class Status : std::uint8_t {
    Stopped,
    Moving,
    NoInput,
};

}

#endif // SIMPLE_JOY_APP_STATUS_HPP
