#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/apm32_base.hpp>
#include <emblib/core.hpp>


namespace mcu {


enum class exec_status {
    ok,
    error,
    busy,
    timeout,
    invalid_argument,
    overflow,
};


inline void fatal_error() {
    emb::fatal_error("mcudrv fatal error");
}


inline void fatal_error(const char* hint, int code = 0) {
    emb::fatal_error(hint, code);
}


} // namespace mcu


#endif
#endif
