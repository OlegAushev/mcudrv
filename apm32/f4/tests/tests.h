#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../chrono/chrono.h"

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
