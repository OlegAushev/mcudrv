#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <emblib/testrunner/testrunner.h>
#include <mcudrv/stm32/h7/system/system.h>
#include <mcudrv/stm32/h7/gpio/gpio.h>
#include <mcudrv/stm32/h7/chrono/chrono.h>
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
