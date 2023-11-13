#pragma once


#ifdef APM32F4xx


#include <apm32f4xx.h>
#include "../mcudef.h"
#include <chrono>


namespace mcu {


inline uint32_t core_clk_freq() { return SystemCoreClock; }


void init();


void delay(std::chrono::milliseconds delay); 


} // namespace mcu


#endif
