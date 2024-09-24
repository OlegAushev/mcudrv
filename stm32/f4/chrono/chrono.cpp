#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/chrono/chrono.h>


/// @brief Mandatory HAL-lib tick handler.
extern "C" void SysTick_Handler()
{
    HAL_IncTick();
    mcu::chrono::steady_clock::on_interrupt();
}


namespace mcu {
namespace chrono {


void steady_clock::init() {
    _initialized = true;
}


void high_resolution_clock::init() {
    if (!steady_clock::initialized()) {
        fatal_error();
    }
    _ticks_usec = core_clk_freq() / 1000000;

    _initialized = true;
}


} //namespace chrono
} // namespace mcu


#endif
#endif
