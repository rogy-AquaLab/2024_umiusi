#include <stdio.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <utility>

#include "nucleo_communicate/recv_data.hpp"
#include "nucleo_communicate/send_data.hpp"
#include "nucleo_communicate/serial_port.hpp"

using nucleo_com::SerialPort;

SerialPort::SerialPort(const std::string& path) : serial(nullptr) {
    this->serial = fopen(path.c_str(), "r+");
}

SerialPort::~SerialPort() {
    fclose(this->serial);
}

void SerialPort::setup() {
    // 9600-8-N-1
    // baud rate: 9600
    // data frame: 8
    // no parity bit
    // stop bit: 1
    termios tio{};
    tio.c_cflag = CREAD | CLOCAL | CS8;
    cfsetspeed(&tio, B9600);
    cfmakeraw(&tio);
    const int fd = fileno(this->serial);
    tcsetattr(fd, TCSANOW, &tio);
    ioctl(fd, TCSETS, &tio);
}

bool SerialPort::send(const SendData& data) {
    std::uint8_t header = 0x00;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    fwrite(
        data.buffer.data(),
        sizeof(SendData::BufferType::value_type),
        data.buffer.size(),
        this->serial
    );
    std::uint8_t res_header = ~header;
    fread(&res_header, sizeof(std::uint8_t), 1, this->serial);
    return res_header == header;
}

auto SerialPort::receive_state() -> std::optional<NucleoState> {
    std::uint8_t header = 0x02;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    std::uint8_t recv_data = 0;
    fread(&recv_data, sizeof(std::uint8_t), 1, this->serial);
    std::uint8_t res_header = ~header;
    fread(&res_header, sizeof(std::uint8_t), 1, this->serial);
    if (res_header != header) {
        return std::nullopt;
    }
    return static_cast<NucleoState>(recv_data);
}

auto SerialPort::receive() -> std::optional<RecvData> {
    std::uint8_t header = 0x01;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    RecvData::BufferType buffer{};
    fread(
        buffer.data(),
        sizeof(RecvData::BufferType::value_type),
        buffer.size(),
        this->serial
    );
    std::uint8_t res_header = ~header;
    fread(&res_header, sizeof(std::uint8_t), 1, this->serial);
    if (res_header != header) {
        return std::nullopt;
    }
    return RecvData(std::move(buffer));
}

bool SerialPort::initialize() {
    std::uint8_t header = 0xFE;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    std::uint8_t res_header = ~header;
    fread(&res_header, sizeof(std::uint8_t), 1, this->serial);
    return res_header == header;
}

bool SerialPort::suspend() {
    std::uint8_t header = 0xFF;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    std::uint8_t res_header = ~header;
    fread(&res_header, sizeof(std::uint8_t), 1, this->serial);
    return res_header == header;
}
