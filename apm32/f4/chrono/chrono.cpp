#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv/apm32/f4/chrono/chrono.hpp>
#include <mcudrv/apm32/f4/system/system.hpp>

extern "C" void SysTick_Handler() {
    // TODO HAL_IncTick();
    mcu::apm32::chrono::steady_clock::on_interrupt();
}

namespace mcu {
namespace apm32 {
namespace chrono {

void steady_clock::init() {
    // init systick
    SysTick_ConfigCLKSource(SYSTICK_CLK_SOURCE_HCLK);

    uint32_t ticks_msec = core_clk_freq() / 1000;
    SysTick->LOAD = ticks_msec - 1;
    // set to highest priority instead of (1UL << __NVIC_PRIO_BITS) - 1UL
    NVIC_SetPriority(SysTick_IRQn, 0);
    SysTick->VAL = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;

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
} // namespace apm32
} // namespace mcu

#endif
#endif
