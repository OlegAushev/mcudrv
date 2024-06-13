#pragma once


#include <mcudrv/apm32/f4/adc/adc.h>
#include <emblib/filter.h>


namespace mcu {


class TemperatureSensor {
public:
    emb::exp_filter<float> filter;
protected:
    TemperatureSensor() : filter(1, 1) {}
public:
    static TemperatureSensor& instance() {
        static TemperatureSensor s;
        return s;
    }
    
    TemperatureSensor(TemperatureSensor const &) = delete;
    TemperatureSensor &operator=(TemperatureSensor const &) = delete;

    static float from_raw_value(uint32_t raw_value) {
        float v = float(raw_value) / 4095.f * 3.3f;
        return (v - 0.7782f) / 0.0024f + 28;
    }
};


} // namespace mcu
