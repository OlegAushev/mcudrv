#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../apm32_base.h"


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
