#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <emblib/testrunner/testrunner.h>
#include "../system/system.h"
#include "../gpio/gpio.h"
#include "../chrono/chrono.h"
#include <mcubsp/stm32/h743_nucleo/leds/leds.h>


namespace mcu {


class tests {
public:
    static void gpio_test();
    static void chrono_test();
};


} // namespace mcu


#endif
#endif
