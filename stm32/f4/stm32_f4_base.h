#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include "../stm32_base.h"


namespace mcu {


enum class DrvStatus {
    ok,
    busy,
    timeout,
    invalid_argument,
    overflow,
    internal_error
};


} // namespace mcu


#endif
#endif
