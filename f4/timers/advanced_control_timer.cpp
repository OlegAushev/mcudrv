#ifdef STM32F4xx

#include <mculib_stm32/f4/timers/advanced_control_timer.h>


namespace mcu {

namespace timers {


AdvancedControlTimer::AdvancedControlTimer(AdvancedControlPeripheral peripheral, const Config& config)
        : emb::interrupt_invoker_array<AdvancedControlTimer, adv_peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk();

    _handle.Instance = impl::adv_timer_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_init;

    if (config.hal_init.Period == 0 && config.freq != 0) {
        // period specified by freq
        switch (config.hal_init.CounterMode) {
        case TIM_COUNTERMODE_UP:
        case TIM_COUNTERMODE_DOWN:
            _handle.Init.Period = ((core_clk_freq() / (config.hal_init.Prescaler+1)) / config.freq) - 1;
            break;
        case TIM_COUNTERMODE_CENTERALIGNED1:
        case TIM_COUNTERMODE_CENTERALIGNED2:
        case TIM_COUNTERMODE_CENTERALIGNED3:
            _handle.Init.Period = (core_clk_freq() / (config.hal_init.Prescaler+1)) / (2 * config.freq);
            break;
        }
    }

    if (HAL_TIM_PWM_Init(&_handle) != HAL_OK) {
        fatal_error("timer initialization failed");
    }

    auto bdt_config = config.bdt_hal_init;
    if (HAL_TIMEx_ConfigBreakDeadTime(&_handle, &bdt_config) != HAL_OK) {
        fatal_error("timer dead-time initialization failed");
    }
}


void AdvancedControlTimer::_enable_clk() {
    auto timer_idx = std::to_underlying(_peripheral);
    if (_clk_enabled[timer_idx]) {
        return;
    }

    impl::adv_timer_clk_enable_funcs[timer_idx]();
    _clk_enabled[timer_idx] = true;
}


void AdvancedControlTimer::init_pwm(Channel channel, ChannelConfig config, ChPin* pin_ch, ChPin* pin_chn) {
    if (HAL_TIM_PWM_ConfigChannel(&_handle, &config.oc_hal_init, std::to_underlying(channel)) != HAL_OK) {
        fatal_error("timer pwm channel initialization failed");
    }

    if (pin_ch) {
        mcu::set_bit(_handle.Instance->CCER, uint32_t(TIM_CCx_ENABLE) << std::to_underlying(channel));
    }

    if (pin_chn) {
        mcu::set_bit(_handle.Instance->CCER, uint32_t(TIM_CCxN_ENABLE) << std::to_underlying(channel));
    }

    mcu::set_bit(_handle.Instance->CR1, TIM_CR1_CEN);
}


} // namespace timers

} // namespace mcu

#endif
