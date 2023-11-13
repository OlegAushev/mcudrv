#pragma once


#ifdef STM32F4xx


#include "../common.h"
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
