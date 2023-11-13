#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include "../stm32_base.h"
#include <cassert>


namespace mcu {


enum class Error {
    none,
    busy,
    timeout,
    invalid_argument,
    overflow,
    internal
};


} // namespace mcu


#endif
#endif
