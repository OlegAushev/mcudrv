#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/stm32_f4_base.h>
#include <emblib/core.hpp>
#include <cstdio>
#include <chrono>


namespace mcu {


void init_hal();


void init_clk();


inline uint32_t core_clk_freq() { return SystemCoreClock; }


void reset_device();


void fatal_error();


void fatal_error(const char* hint, int code = 0);


inline void enable_interrupts() { __enable_irq(); }


inline void disable_interrupts() { __disable_irq(); }


class critical_section {
private:
    bool irq_enabled;
public:
    critical_section() {
        irq_enabled = (__get_PRIMASK() == 0);
        __disable_irq();
    }

    ~critical_section() {
        if (irq_enabled) {
            __enable_irq();
        }
    }
};


// inline float calculate_mcu_temperature(uint32_t adcData) {
//     return float(int32_t(adcData) - int32_t(*TEMPSENSOR_CAL1_ADDR)) * (float(TEMPSENSOR_CAL2_TEMP) - float(TEMPSENSOR_CAL1_TEMP))
//             / (int32_t(*TEMPSENSOR_CAL2_ADDR) - int32_t(*TEMPSENSOR_CAL1_ADDR)) + float(TEMPSENSOR_CAL1_TEMP); 
// }


inline float calculate_mcu_vref(uint32_t adc_data) {
    return 3.3f * float(*VREFINT_CAL_ADDR) / float(adc_data);
}


} // namespace mcu


#endif
#endif
