#pragma once


#ifdef MCUDRV_GD32
#ifdef GD32F4xx


#include "../gd32_base.h"


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
