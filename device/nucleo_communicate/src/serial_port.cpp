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

void SerialPort::send(const SendData& data) {
    std::uint8_t header = 0x00;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    fwrite(
        data.buffer.data(),
        sizeof(SendData::BufferType::value_type),
        data.buffer.size(),
        this->serial
    );
}

auto SerialPort::receive() -> RecvData {
    std::uint8_t header = 0x01;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
    RecvData::BufferType buffer{};
    fread(
        buffer.data(),
        sizeof(RecvData::BufferType::value_type),
        buffer.size(),
        this->serial
    );
    return RecvData(std::move(buffer));
}

void SerialPort::quit() {
    std::uint8_t header = 0xFF;
    fwrite(&header, sizeof(std::uint8_t), 1, this->serial);
}
