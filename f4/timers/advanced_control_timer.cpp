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


void AdvancedControlTimer::init_pwm(Channel channel, ChannelConfig config, const PinConfig* pin_ch_config, const PinConfig* pin_chn_config) {
    if (pin_ch_config) {
        mcu::gpio::Output({
            .port = pin_ch_config->port, 
            .pin = {
                .Pin = pin_ch_config->pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                .Alternate = pin_ch_config->af_selection
            },
            .active_state = emb::gpio::ActiveState::high});
    }
    
    if (pin_chn_config) {
        mcu::gpio::Output({
            .port = pin_chn_config->port, 
            .pin = {
                .Pin = pin_chn_config->pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                .Alternate = pin_chn_config->af_selection
            },
            .active_state = emb::gpio::ActiveState::high});
    }

    if (HAL_TIM_PWM_ConfigChannel(&_handle, &config.oc_hal_init, std::to_underlying(channel)) != HAL_OK) {
        fatal_error("timer pwm channel initialization failed");
    }

    //TIM_CCxChannelCmd(_handle.Instance, std::to_underlying(channel), TIM_CCx_ENABLE);
    //TIM_CCxNChannelCmd(_handle.Instance, std::to_underlying(channel), TIM_CCxN_ENABLE);
}


} // namespace timers

} // namespace mcu

#endif
