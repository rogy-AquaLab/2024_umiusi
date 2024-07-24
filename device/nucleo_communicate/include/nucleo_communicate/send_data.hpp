#ifndef NUCLEO_COMMUNICATE_SEND_DATA_HPP
#define NUCLEO_COMMUNICATE_SEND_DATA_HPP

#include <array>
#include <cstdint>

#include "packet_interfaces/msg/power.hpp"

namespace nucleo_com {

struct SendData {
    using BufferType = std::array<std::uint8_t, 16>;

    BufferType buffer;

    SendData() = delete;

    explicit SendData(BufferType&& buf);

    static auto from_msg(const packet_interfaces::msg::Power& msg) -> SendData;
};

}

#endif // NUCLEO_COMMUNICATE_SEND_DATA_HPP
