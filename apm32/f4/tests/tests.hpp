#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.hpp>
#include <mcudrv/apm32/f4/gpio/gpio.hpp>
#include <mcudrv/apm32/f4/chrono/chrono.hpp>

#include <emblib/testrunner/testrunner.hpp>


namespace mcu {


class tests {
public:
    static void gpio_test();
    static void chrono_test();
};


} // namespace mcu


#endif
#endif
