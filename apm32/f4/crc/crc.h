#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/apm32f4_base.h>
#include <apm32f4xx_crc.h>
#include <cstddef>


namespace mcu {


namespace crc {


inline uint32_t calc_crc32(const uint8_t* buf, size_t len) {
    return 42; // TODO
}


} // namespace crc


} // namespace mcu


#endif
#endif
