#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/chrono/chrono.h>


extern "C" void SysTick_Handler()
{
    // TODO HAL_IncTick();
    mcu::chrono::steady_clock::on_interrupt();
}


namespace mcu {


namespace chrono {


void steady_clock::initialize() {
    // init systick
    SysTick_ConfigCLKSource(SYSTICK_CLK_SOURCE_HCLK);

    uint32_t ticks = core_clk_freq() / 1000;
    SysTick->LOAD = ticks - 1;
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
    SysTick->VAL = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}


} //namespace chrono


} // namespace mcu


#endif
#endif
