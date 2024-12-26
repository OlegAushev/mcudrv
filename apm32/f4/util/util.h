#pragma once

#include <mcudrv/apm32/f4/adc/adc.h>

#include <emblib/filter.hpp>

namespace mcu {
namespace apm32 {
namespace util {

class temperature_sensor {
public:
    static float get_value(uint32_t raw_value) {
        float v = float(raw_value) / 4095.f * 3.3f;
        return (v - 0.7782f) / 0.0024f + 28;
    }
};

} // namespace util
} // namespace apm32
} // namespace mcu
