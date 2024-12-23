#pragma once


#ifdef MCUDRV_C28X


#include <tests/tests_config.hpp>
#include <emblib/testrunner/testrunner.hpp>
#include <mcudrv/c28x/f2837xd/system/system.hpp>
#include <mcudrv/c28x/f2837xd/gpio/gpio.hpp>
#include <mcudrv/c28x/f2837xd/chrono/chrono.hpp>
#include <mcudrv/c28x/f2837xd/crc/crc.hpp>

#ifdef _LAUNCHXL_F28379D
#include <mcubsp/c28x/launchpad/launchpad.hpp>
#endif


namespace mcu {
namespace c28x {


class tests {
public:
    static void gpio_test();
    static void chrono_test();
    static void crc_test();
};


} // namespace c28x
} // namespace mcu


#endif
