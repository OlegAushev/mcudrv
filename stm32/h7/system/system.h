#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include <emblib/core.h>
#include <cstdio>
#include <chrono>


//void MX_GPIO_Init();


namespace mcu {


void init_hal();


void init_device_clk();


inline uint32_t core_clk_freq() { return SystemCoreClock; }
inline uint32_t d2_clk_freq() { return SystemD2Clock; }


void reset_device();


void fatal_error(const char* hint, int code = 0);


inline void delay(std::chrono::milliseconds delay) {
    HAL_Delay(static_cast<uint32_t>(delay.count()));
}


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


inline void enable_icache() { SCB_EnableICache(); }


inline void disable_icache() { SCB_DisableICache(); }


inline void enable_dcache() { SCB_EnableDCache(); }


inline void disable_dcache() { SCB_DisableDCache(); }


inline float calculate_mcu_temperature(uint32_t adcData) {
    return float(int32_t(adcData) - int32_t(*TEMPSENSOR_CAL1_ADDR)) * (float(TEMPSENSOR_CAL2_TEMP) - float(TEMPSENSOR_CAL1_TEMP))
            / (int32_t(*TEMPSENSOR_CAL2_ADDR) - int32_t(*TEMPSENSOR_CAL1_ADDR)) + float(TEMPSENSOR_CAL1_TEMP); 
}


inline float calculate_mcu_vref(uint32_t adcData) {
    return 3.3f * float(*VREFINT_CAL_ADDR) / float(adcData);
}


} // namespace mcu


#endif
#endif
