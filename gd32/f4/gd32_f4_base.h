#pragma once


#ifdef MCUDRV_GD32
#ifdef GD32F4xx


#include "../gd32_base.h"


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
