#ifndef NUCLEO_COMMUNICATE_NUCLEO_STATE_HPP
#define NUCLEO_COMMUNICATE_NUCLEO_STATE_HPP

#include <cstdint>

namespace nucleo_com {

// https://github.com/rogy-AquaLab/2024_umiusi_nucleo/blob/4ccdc0d/include/umiusi/state.hpp

enum class NucleoState: std::uint8_t {
    INITIALIZING = 0,
    SUSPEND = 1,
    RUNNING = 2,
};

}

#endif // NUCLEO_COMMUNICATE_NUCLEO_STATE_HPP
