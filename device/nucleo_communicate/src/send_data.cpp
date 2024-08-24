#include "nucleo_communicate/send_data.hpp"

using nucleo_com::SendData;

SendData::SendData(SendData::BufferType&& buf) : buffer(buf) {}

auto SendData::from_msg(const packet_interfaces::msg::Power& msg) -> SendData {
    return SendData(
        {
            static_cast<std::uint8_t>((msg.bldc[0] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[0] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[1] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[1] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[2] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[2] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[3] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.bldc[3] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[0] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[0] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[1] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[1] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[2] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[2] >> 8) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[3] >> 0) & 0xFF),
            static_cast<std::uint8_t>((msg.servo[3] >> 8) & 0xFF),
        }
    );
}
