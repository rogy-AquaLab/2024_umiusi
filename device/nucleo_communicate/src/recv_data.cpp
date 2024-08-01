#include "nucleo_communicate/recv_data.hpp"

using nucleo_com::RecvData;

RecvData::RecvData(RecvData::BufferType&& buf) : buffer(buf) {}

auto RecvData::flex1_value() const -> std::uint16_t {
    return (static_cast<std::uint16_t>(this->buffer[0]) << 0)
           | (static_cast<std::uint16_t>(this->buffer[1]) << 8);
}

auto RecvData::flex2_value() const -> std::uint16_t {
    return (static_cast<std::uint16_t>(this->buffer[2]) << 0)
           | (static_cast<std::uint16_t>(this->buffer[3]) << 8);
}

auto RecvData::current_value() const -> std::uint16_t {
    return (static_cast<std::uint16_t>(this->buffer[4]) << 0)
           | (static_cast<std::uint16_t>(this->buffer[5]) << 8);
}

auto RecvData::voltage_value() const -> std::uint16_t {
    return (static_cast<std::uint16_t>(this->buffer[6]) << 0)
           | (static_cast<std::uint16_t>(this->buffer[7]) << 8);
}
