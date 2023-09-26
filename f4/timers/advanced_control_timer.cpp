#ifdef STM32F4xx

#include <mculib_stm32/f4/timers/advanced_control_timer.h>


namespace mcu {

namespace timers {


AdvancedControlTimer::AdvancedControlTimer(AdvancedControlPeripheral peripheral, const Config& config)
        : emb::interrupt_invoker_array<AdvancedControlTimer, adv_timer_peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk();

    _handle.Instance = impl::adv_timer_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_base_config;

    _reg = _handle.Instance;

    if (config.hal_base_config.Period == 0 && config.freq != 0) {
        // period specified by freq
        _freq = config.freq;
        float timebase_freq = static_cast<float>(core_clk_freq()) / static_cast<float>(config.hal_base_config.Prescaler+1);

        switch (config.hal_base_config.CounterMode) {
        case TIM_COUNTERMODE_UP:
        case TIM_COUNTERMODE_DOWN:
            _handle.Init.Period = static_cast<uint32_t>((timebase_freq / config.freq) - 1);
            break;
        case TIM_COUNTERMODE_CENTERALIGNED1:
        case TIM_COUNTERMODE_CENTERALIGNED2:
        case TIM_COUNTERMODE_CENTERALIGNED3:
            _handle.Init.Period = static_cast<uint32_t>((timebase_freq / config.freq) / 2);
            break;
        }
    }

    switch (config.hal_base_config.ClockDivision) {
    case TIM_CLOCKDIVISION_DIV1:
        _t_dts_ns = static_cast<float>(config.hal_base_config.Prescaler+1) * 1000000000.f / static_cast<float>(core_clk_freq());
        break;
    case TIM_CLOCKDIVISION_DIV2:
        _t_dts_ns = 2 * static_cast<float>(config.hal_base_config.Prescaler+1) * 1000000000.f / static_cast<float>(core_clk_freq());
        break;
    case TIM_CLOCKDIVISION_DIV4:
        _t_dts_ns = 4 * static_cast<float>(config.hal_base_config.Prescaler+1) * 1000000000.f / static_cast<float>(core_clk_freq());
        break;
    default:
        fatal_error("timer initialization failed");
        break;
    }

    if (HAL_TIM_PWM_Init(&_handle) != HAL_OK) {
        fatal_error("timer initialization failed");
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
    if (HAL_TIM_PWM_ConfigChannel(&_handle, &config.hal_oc_config, std::to_underlying(channel)) != HAL_OK) {
        fatal_error("timer pwm channel initialization failed");
    }

    if (pin_ch) {
        mcu::set_bit(_reg->CCER, uint32_t(TIM_CCx_ENABLE) << std::to_underlying(channel));
    }

    if (pin_chn) {
        mcu::set_bit(_reg->CCER, uint32_t(TIM_CCxN_ENABLE) << std::to_underlying(channel));
    }

    mcu::set_bit(_reg->CR1, TIM_CR1_CEN);
}


void AdvancedControlTimer::init_bdt(BdtConfig config, BkinPin* pin_bkin) {
    if (config.hal_bdt_config.DeadTime == 0 && config.deadtime_ns != 0) {
        // deadtime specified by deadtime_ns
        if (config.deadtime_ns <= 0X7F * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = static_cast<uint32_t>(config.deadtime_ns / _t_dts_ns);
        } else if (config.deadtime_ns <= 127 * 2 * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = static_cast<uint32_t>((config.deadtime_ns - 64 * 2 * _t_dts_ns) / (2 * _t_dts_ns));
            config.hal_bdt_config.DeadTime |= 0x80;
        } else if (config.deadtime_ns <= 63 * 8 * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = static_cast<uint32_t>((config.deadtime_ns - 32 * 8 * _t_dts_ns) / (8 * _t_dts_ns));
            config.hal_bdt_config.DeadTime |= 0xC0;
        } else if (config.deadtime_ns <= 63 * 16 * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = static_cast<uint32_t>((config.deadtime_ns - 32 * 16 * _t_dts_ns) / (16 * _t_dts_ns));
            config.hal_bdt_config.DeadTime |= 0xE0;
        } else {
            fatal_error("timer dead-time initialization failed");
        }
    }

    if (HAL_TIMEx_ConfigBreakDeadTime(&_handle, &config.hal_bdt_config) != HAL_OK) {
        fatal_error("timer dead-time initialization failed");
    }
}


void AdvancedControlTimer::init_update_interrupts(IrqPriority priority) {
    mcu::set_bit(_reg->DIER, TIM_DIER_UIE);
    mcu::set_irq_priority(impl::adv_timer_up_irqn[std::to_underlying(_peripheral)], priority);
}


void AdvancedControlTimer::init_break_interrupts(IrqPriority priority) {
    mcu::set_bit(_reg->DIER, TIM_DIER_BIE);
    mcu::set_irq_priority(impl::adv_timer_brk_irqn[std::to_underlying(_peripheral)], priority);
    _brk_enabled = true;
}


} // namespace timers

} // namespace mcu


#endif
