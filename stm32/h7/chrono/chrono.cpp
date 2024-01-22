#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/chrono/chrono.h>


/// @brief Mandatory HAL-lib tick handler.
extern "C" void SysTick_Handler()
{
    HAL_IncTick();
    mcu::chrono::steady_clock::on_interrupt();
}


namespace mcu {


namespace chrono {


void steady_clock::init() {

}


} //namespace chrono


} // namespace mcu


#endif
#endif
