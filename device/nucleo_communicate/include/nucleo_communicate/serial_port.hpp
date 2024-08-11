#ifndef NUCLEO_COMMUNICATE_SERIAL_PORT_HPP
#define NUCLEO_COMMUNICATE_SERIAL_PORT_HPP

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
    void send(const nucleo_com::SendData& data);

    auto receive_state() -> nucleo_com::NucleoState;

    auto receive() -> nucleo_com::RecvData;

    void quit();
};

}

#endif // NUCLEO_COMMUNICATE_SERIAL_PORT_HPP
