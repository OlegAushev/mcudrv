#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "advanced_control_timer.h"


namespace mcu {


namespace timers {


AdvancedControlTimer::AdvancedControlTimer(AdvancedControlPeripheral peripheral, Config config)
        : emb::interrupt_invoker_array<AdvancedControlTimer, adv_timer_peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk(peripheral);
    _reg = impl::adv_timer_instances[std::to_underlying(_peripheral)];

    if (config.hal_base_config.period == 0 && config.freq != 0) {
        // period specified by freq
        _freq = config.freq;
        float timebase_freq = static_cast<float>(core_clk_freq()) / static_cast<float>(config.hal_base_config.division+1);

        switch (config.hal_base_config.countMode) {
        case TMR_COUNTER_MODE_UP:
        case TMR_COUNTER_MODE_DOWN:
            config.hal_base_config.period = static_cast<uint32_t>((timebase_freq / config.freq) - 1);
            break;
        case TMR_COUNTER_MODE_CENTER_ALIGNED1:
        case TMR_COUNTER_MODE_CENTER_ALIGNED2:
        case TMR_COUNTER_MODE_CENTER_ALIGNED3:
            config.hal_base_config.period = static_cast<uint32_t>((timebase_freq / config.freq) / 2);
            break;
        }
    }

    switch (config.hal_base_config.clockDivision) {
    case TMR_CLOCK_DIV_1:
        _t_dts_ns = static_cast<float>(config.hal_base_config.division+1) * 1000000000.f / static_cast<float>(core_clk_freq());
        break;
    case TMR_CLOCK_DIV_2:
        _t_dts_ns = 2 * static_cast<float>(config.hal_base_config.division+1) * 1000000000.f / static_cast<float>(core_clk_freq());
        break;
    case TMR_CLOCK_DIV_4:
        _t_dts_ns = 4 * static_cast<float>(config.hal_base_config.division+1) * 1000000000.f / static_cast<float>(core_clk_freq());
        break;
    }

    if (config.arr_preload) {
        _reg->CTRL1_B.ARPEN = 1;
    }
    TMR_ConfigTimeBase(_reg, &config.hal_base_config);
}


void AdvancedControlTimer::_enable_clk(AdvancedControlPeripheral peripheral) {
    auto timer_idx = std::to_underlying(peripheral);
    if (_clk_enabled[timer_idx]) {
        return;
    }

    impl::adv_timer_clk_enable_funcs[timer_idx]();
    _clk_enabled[timer_idx] = true;
}


void AdvancedControlTimer::init_pwm(Channel channel, ChPin* pin_ch, ChPin* pin_chn, ChannelConfig config) {
    switch (channel) {
    case Channel::channel1:
        _reg->CCM1_COMPARE_B.OC1PEN = config.oc_preload;
        TMR_ConfigOC1(_reg, &config.hal_oc_config);
        if (pin_ch) { _reg->CCEN_B.CC1EN = 1; } else { _reg->CCEN_B.CC1EN = 0; }
        if (pin_chn) { _reg->CCEN_B.CC1NEN = 1; } else { _reg->CCEN_B.CC1NEN = 0; }
        break;
    case Channel::channel2:
        _reg->CCM1_COMPARE_B.OC2PEN = config.oc_preload;
        TMR_ConfigOC2(_reg, &config.hal_oc_config);
        if (pin_ch) { _reg->CCEN_B.CC2EN = 1; } else { _reg->CCEN_B.CC2EN = 0; }
        if (pin_chn) { _reg->CCEN_B.CC2NEN = 1; } else { _reg->CCEN_B.CC2NEN = 0; }
        break;
    case Channel::channel3:
        _reg->CCM2_COMPARE_B.OC3PEN = config.oc_preload;
        TMR_ConfigOC3(_reg, &config.hal_oc_config);
        if (pin_ch) { _reg->CCEN_B.CC3EN = 1; } else { _reg->CCEN_B.CC3EN = 0; }
        if (pin_chn) { _reg->CCEN_B.CC3NEN = 1; } else { _reg->CCEN_B.CC3NEN = 0; }
        break;
    case Channel::channel4:
        _reg->CCM2_COMPARE_B.OC4PEN = config.oc_preload;
        TMR_ConfigOC4(_reg, &config.hal_oc_config);
        if (pin_ch) { _reg->CCEN_B.CC4EN = 1; } else { _reg->CCEN_B.CC4EN = 0; }
        break;
    }
}


void AdvancedControlTimer::init_bdt(BkinPin* pin_bkin, BdtConfig config) {
    if (config.hal_bdt_config.deadTime == 0 && config.deadtime_ns != 0) {
        // deadtime specified by deadtime_ns
        if (config.deadtime_ns <= 0X7F * _t_dts_ns) {
            config.hal_bdt_config.deadTime = static_cast<uint16_t>(config.deadtime_ns / _t_dts_ns);
        } else if (config.deadtime_ns <= 127 * 2 * _t_dts_ns) {
            config.hal_bdt_config.deadTime = static_cast<uint16_t>((config.deadtime_ns - 64 * 2 * _t_dts_ns) / (2 * _t_dts_ns));
            config.hal_bdt_config.deadTime |= 0x80;
        } else if (config.deadtime_ns <= 63 * 8 * _t_dts_ns) {
            config.hal_bdt_config.deadTime = static_cast<uint16_t>((config.deadtime_ns - 32 * 8 * _t_dts_ns) / (8 * _t_dts_ns));
            config.hal_bdt_config.deadTime |= 0xC0;
        } else if (config.deadtime_ns <= 63 * 16 * _t_dts_ns) {
            config.hal_bdt_config.deadTime = static_cast<uint16_t>((config.deadtime_ns - 32 * 16 * _t_dts_ns) / (16 * _t_dts_ns));
            config.hal_bdt_config.deadTime |= 0xE0;
        } else {
            fatal_error("timer dead-time initialization failed");
        }
    }

    TMR_ConfigBDT(_reg, &config.hal_bdt_config);
}


void AdvancedControlTimer::init_update_interrupts(IrqPriority priority) {
    _reg->DIEN_B.UIEN = 1;
    set_irq_priority(impl::adv_timer_up_irqn[std::to_underlying(_peripheral)], priority);
}


void AdvancedControlTimer::init_break_interrupts(IrqPriority priority) {
    _reg->DIEN_B.BRKIEN = 1;
    set_irq_priority(impl::adv_timer_brk_irqn[std::to_underlying(_peripheral)], priority);
    _brk_enabled = true;
}


} // namespace timers


} // namepsace mcu


#endif
#endif
