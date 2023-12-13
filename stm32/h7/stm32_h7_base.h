#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_base.h"


namespace mcu {


enum class DrvStatus {
    ok,
    error,
    busy,
    timeout,
    invalid_argument,
    overflow,
};


} // namespace mcu


#endif
#endif
