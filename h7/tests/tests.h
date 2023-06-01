#pragma once


#include <emblib_stm32/testrunner/testrunner.h>

#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../chrono/chrono.h"
#include <bsp/nucleo_h743/leds/leds.h>


namespace mcu {

class tests {
public:
    static void gpio_test();
    static void chrono_test();
};

} // namespace mcu

