#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../apm32_f4_base.h"
#include <apm32f4xx.h>
#include <chrono>


namespace mcu {


inline uint32_t core_clk_freq() { return SystemCoreClock; }


void init();


void delay(std::chrono::milliseconds delay); 


} // namespace mcu


#endif
#endif
