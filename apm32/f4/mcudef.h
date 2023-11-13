#pragma once


#ifdef APM32F4xx


#include "../common.h"


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
