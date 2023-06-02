#ifdef STM32H7xx

#include <mculib_stm32/h7/timers/timers_gp.h>


namespace mcu {

namespace timers_gp {

Timer::Timer(Peripheral peripheral, const Config& config)
        : emb::interrupt_invoker_array<Timer, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    enable_clk();

    _handle.Instance = impl::timer_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_init;

    if (config.hal_init.Period == 0 && config.freq != 0) {
        // period specified by freq
        switch (config.hal_init.CounterMode) {
        case TIM_COUNTERMODE_UP:
        case TIM_COUNTERMODE_DOWN:
            _handle.Init.Period = ((d2_clk_freq() / (config.hal_init.Prescaler+1)) / config.freq) - 1;
            break;
        case TIM_COUNTERMODE_CENTERALIGNED1:
        case TIM_COUNTERMODE_CENTERALIGNED2:
        case TIM_COUNTERMODE_CENTERALIGNED3:
            _handle.Init.Period = (d2_clk_freq() / (config.hal_init.Prescaler+1)) / (2 * config.freq);
            break;
        }
    }

    if (HAL_TIM_PWM_Init(&_handle) != HAL_OK) {
        fatal_error("timer initialization failed");
    }
}

}

} // namespace mcu

#endif

