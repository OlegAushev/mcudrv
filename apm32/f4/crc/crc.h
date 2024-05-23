#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.h>
#include <apm32f4xx_rcm.h>
#include <apm32f4xx_crc.h>
#include <cstddef>


namespace mcu {
namespace crc {


inline void init() {
    RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_CRC);
}


inline uint32_t calc_crc32(const uint8_t* buf, size_t len) {
    CRC->CTRL_B.RST = BIT_SET;
    for (auto i = 0uz; i < len; ++i) {
        CRC->DATA = buf[i];
    }
    return CRC->DATA;
}


} // namespace crc
} // namespace mcu


#endif
#endif
