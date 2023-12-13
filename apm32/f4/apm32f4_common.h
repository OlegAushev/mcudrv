#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../apm32_common.h"


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
