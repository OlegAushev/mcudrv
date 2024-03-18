#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <mcudrv/apm32/f4/chrono/chrono.h>

#include <emblib/testrunner/testrunner.h>


namespace mcu {


class tests {
public:
    static void gpio_test();
    static void chrono_test();
};


} // namespace mcu


#endif
#endif
