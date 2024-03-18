#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/timers/advanced_control/pwm.h>


namespace mcu {


namespace timers {


namespace adv {


PwmTimer::PwmTimer(Peripheral peripheral, const PwmConfig& config)
        : impl::AbstractTimer(peripheral, OpMode::pwm_generation) 
{
    _handle.Init = config.hal_base_config;

    float timebase_freq = float(core_clk_freq()) / float(config.hal_base_config.Prescaler+1);
    
    if (config.hal_base_config.Period == 0 && config.freq != 0) {
        // period specified by freq
        _freq = config.freq;
        switch (config.hal_base_config.CounterMode) {
        case TIM_COUNTERMODE_UP:
        case TIM_COUNTERMODE_DOWN:
            _handle.Init.Period = uint32_t((timebase_freq / config.freq) - 1);
            break;
        case TIM_COUNTERMODE_CENTERALIGNED1:
        case TIM_COUNTERMODE_CENTERALIGNED2:
        case TIM_COUNTERMODE_CENTERALIGNED3:
            _handle.Init.Period = uint32_t((timebase_freq / config.freq) / 2);
            break;
        }
    } else if (config.hal_base_config.Period != 0) {
        // period specified by hal_base_config.Period
        switch (config.hal_base_config.CounterMode) {
        case TIM_COUNTERMODE_UP:
        case TIM_COUNTERMODE_DOWN:
            _freq = timebase_freq / float(_handle.Init.Period + 1);
            break;
        case TIM_COUNTERMODE_CENTERALIGNED1:
        case TIM_COUNTERMODE_CENTERALIGNED2:
        case TIM_COUNTERMODE_CENTERALIGNED3:
            _freq = timebase_freq / float(_handle.Init.Period * 2);
            break;
        }
    } else {
        fatal_error("invalid config");
    }

    switch (config.hal_base_config.ClockDivision) {
    case TIM_CLOCKDIVISION_DIV1:
        _t_dts_ns = float(config.hal_base_config.Prescaler+1) * 1000000000.f / float(core_clk_freq());
        break;
    case TIM_CLOCKDIVISION_DIV2:
        _t_dts_ns = 2 * float(config.hal_base_config.Prescaler+1) * 1000000000.f / float(core_clk_freq());
        break;
    case TIM_CLOCKDIVISION_DIV4:
        _t_dts_ns = 4 * float(config.hal_base_config.Prescaler+1) * 1000000000.f / float(core_clk_freq());
        break;
    default:
        fatal_error("timer initialization failed");
        break;
    }

    if (HAL_TIM_PWM_Init(&_handle) != HAL_OK) {
        fatal_error("timer initialization failed");
    }
}


void PwmTimer::initialize_channel(Channel channel, ChPin* pin_ch, ChPin* pin_chn, ChannelConfig config) {
    if (HAL_TIM_PWM_ConfigChannel(&_handle, &config.hal_oc_config, std::to_underlying(channel)) != HAL_OK) {
        fatal_error("timer pwm channel initialization failed");
    }

    if (pin_ch) {
        set_bit(_reg->CCER, uint32_t(TIM_CCx_ENABLE) << std::to_underlying(channel));
    }

    if (pin_chn) {
        set_bit(_reg->CCER, uint32_t(TIM_CCxN_ENABLE) << std::to_underlying(channel));
    }
}


void PwmTimer::initialize_bdt(BkinPin* pin_bkin, BdtConfig config) {
    if (config.hal_bdt_config.DeadTime == 0) {
        // deadtime specified by deadtime_ns
        _deadtime = config.deadtime_ns * 1E-09f;
        if (config.deadtime_ns <= 0X7F * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = uint32_t(config.deadtime_ns / _t_dts_ns);
        } else if (config.deadtime_ns <= 127 * 2 * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = uint32_t((config.deadtime_ns - 64 * 2 * _t_dts_ns) / (2 * _t_dts_ns));
            config.hal_bdt_config.DeadTime |= 0x80;
        } else if (config.deadtime_ns <= 63 * 8 * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = uint32_t((config.deadtime_ns - 32 * 8 * _t_dts_ns) / (8 * _t_dts_ns));
            config.hal_bdt_config.DeadTime |= 0xC0;
        } else if (config.deadtime_ns <= 63 * 16 * _t_dts_ns) {
            config.hal_bdt_config.DeadTime = uint32_t((config.deadtime_ns - 32 * 16 * _t_dts_ns) / (16 * _t_dts_ns));
            config.hal_bdt_config.DeadTime |= 0xE0;
        } else {
            fatal_error("timer dead-time initialization failed");
        }
    } else {
        auto dtg = config.hal_bdt_config.DeadTime;
        if ((dtg & 0x80) == 0) {
            _deadtime = float(dtg) * _t_dts_ns * 1E-09f;
        } else if ((dtg & 0xC0) == 0x80) {
            _deadtime = float(64 + (dtg & 0x3F)) * 2 * _t_dts_ns * 1E-09f;
        } else if ((dtg & 0xE0) == 0xC0) {
            _deadtime = float(32 + (dtg & 0x1F)) * 8 * _t_dts_ns * 1E-09f;
        } else if ((dtg & 0xE0) == 0xE0) {
            _deadtime = float(32 + (dtg & 0x1F)) * 16 * _t_dts_ns * 1E-09f;
        } else {
            fatal_error("timer dead-time initialization failed");
        }
    }

    if (HAL_TIMEx_ConfigBreakDeadTime(&_handle, &config.hal_bdt_config) != HAL_OK) {
        fatal_error("timer dead-time initialization failed");
    }
}


void PwmTimer::initialize_update_interrupts(IrqPriority priority) {
    set_bit<uint32_t>(_reg->DIER, TIM_DIER_UIE);
    set_irq_priority(impl::up_irq_nums[std::to_underlying(_peripheral)], priority);
}


void PwmTimer::initialize_break_interrupts(IrqPriority priority) {
    set_bit<uint32_t>(_reg->DIER, TIM_DIER_BIE);
    set_irq_priority(impl::brk_irq_nums[std::to_underlying(_peripheral)], priority);
    _brk_enabled = true;
}


} // namespace adv


} // namespace timers


} // namespace mcu


#endif
#endif
