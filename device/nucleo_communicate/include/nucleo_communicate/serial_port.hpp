#ifndef NUCLEO_COMMUNICATE_SERIAL_PORT_HPP
#define NUCLEO_COMMUNICATE_SERIAL_PORT_HPP

#include <optional>
#include <stdio.h>
#include <string>
#include <termios.h>

#include "nucleo_communicate/nucleo_state.hpp"
#include "nucleo_communicate/recv_data.hpp"
#include "nucleo_communicate/send_data.hpp"

namespace nucleo_com {

class SerialPort {
private:
    FILE* serial;

public:
    explicit SerialPort(const std::string& path = "/dev/ttyACM0");

    ~SerialPort();

    // same as
    // https://os.mbed.com/docs/mbed-os/v6.16/apis/serial-uart-apis.html#configuration
    void setup();

    // https://github.com/rogy-AquaLab/2024_umiusi_nucleo
    bool send(const nucleo_com::SendData& data);

    auto receive_state() -> std::optional<nucleo_com::NucleoState>;

    auto receive() -> std::optional<nucleo_com::RecvData>;

    /// nucleoに初期化命令を送る *SerialPortの初期化ではない
    bool initialize();

    bool suspend();
};

}

#endif // NUCLEO_COMMUNICATE_SERIAL_PORT_HPP
