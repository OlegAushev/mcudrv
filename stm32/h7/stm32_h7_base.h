#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


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


using HalStatus = HAL_StatusTypeDef;


} // namespace mcu


#endif
#endif
