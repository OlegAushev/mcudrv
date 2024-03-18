#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx

#include <emblib/testrunner/testrunner.h>

#include <mcudrv/stm32/f4/system/system.h>
#include <mcudrv/stm32/f4/gpio/gpio.h>
#include <mcudrv/stm32/f4/chrono/chrono.h>
#include <mcubsp/stm32/f446_nucleo/leds/leds.h>


namespace mcu {


class tests {
public:
    static void gpio_test();
    static void chrono_test();
};


} // namespace mcu


#endif
#endif
