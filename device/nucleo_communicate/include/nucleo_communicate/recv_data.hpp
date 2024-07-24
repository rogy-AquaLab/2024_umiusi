#ifndef NUCLEO_COMMUNICATE_RECV_DATA_HPP
#define NUCLEO_COMMUNICATE_RECV_DATA_HPP

#include <array>
#include <cstdint>
#include <std_msgs/msg/detail/header__struct.hpp>

namespace nucleo_com {

struct RecvData {
    using BufferType = std::array<std::uint8_t, 8>;

    BufferType buffer;

    explicit RecvData(BufferType&& buf);

    auto flex1_value() const -> std::uint16_t;
    auto flex2_value() const -> std::uint16_t;
    auto current_value() const -> std::uint16_t;
    auto voltage_value() const -> std::uint16_t;
};

}

#endif // NUCLEO_COMMUNICATE_RECV_DATA_HPP
