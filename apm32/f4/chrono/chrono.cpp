#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/chrono/chrono.h>


extern "C" void SysTick_Handler()
{
    // TODO HAL_IncTick();
    mcu::chrono::system_clock::on_interrupt();
}


namespace mcu {


namespace chrono {


void system_clock::init() {
    // init systick
    SysTick_ConfigCLKSource(SYSTICK_CLK_SOURCE_HCLK);

    uint32_t ticks = core_clk_freq() / 1000;
    SysTick->LOAD = ticks - 1;
    NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
    SysTick->VAL = 0UL;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    set_initialized();
}


void system_clock::run_tasks() {
    for (size_t i = 0; i < _tasks.size(); ++i) {
        if (now() >= (_tasks[i].timepoint + _tasks[i].period)) {
            if (_tasks[i].func(i) == TaskStatus::success) {
                _tasks[i].timepoint = now();
            }
        }
    }


    if (_delayed_task_delay.count() != 0)
    {
        if (now() >= (_delayed_task_start + _delayed_task_delay))
        {
            _delayed_task();
            _delayed_task_delay = std::chrono::milliseconds(0);
        }
    }
}


} //namespace chrono


} // namespace mcu


#endif
#endif
