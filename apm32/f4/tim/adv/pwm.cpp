#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv/apm32/f4/tim/adv/pwm.h>

namespace mcu {
namespace apm32 {
namespace tim {
namespace adv {

PwmTimer::PwmTimer(Peripheral peripheral,
                   const PwmConfig& config,
                   BkinPin* pin_bkin)
        : impl::AbstractTimer(peripheral, OpMode::pwm_generation) {
    auto cfg = config;

    float timebase_freq =
            float(core_clk_freq()) / float(config.hal_base_config.division + 1);

    if (cfg.hal_base_config.period == 0 && cfg.freq != 0) {
        // period specified by freq
        _freq = config.freq;
        switch (config.hal_base_config.countMode) {
        case TMR_COUNTER_MODE_UP:
        case TMR_COUNTER_MODE_DOWN:
            cfg.hal_base_config.period =
                    uint32_t((timebase_freq / config.freq) - 1);
            break;
        case TMR_COUNTER_MODE_CENTER_ALIGNED1:
        case TMR_COUNTER_MODE_CENTER_ALIGNED2:
        case TMR_COUNTER_MODE_CENTER_ALIGNED3:
            cfg.hal_base_config.period =
                    uint32_t((timebase_freq / config.freq) / 2);
            break;
        }
    } else if (config.hal_base_config.period != 0) {
        // period specified by hal_base_config.period
        switch (config.hal_base_config.countMode) {
        case TMR_COUNTER_MODE_UP:
        case TMR_COUNTER_MODE_DOWN:
            _freq = timebase_freq / float(cfg.hal_base_config.period + 1);
            break;
        case TMR_COUNTER_MODE_CENTER_ALIGNED1:
        case TMR_COUNTER_MODE_CENTER_ALIGNED2:
        case TMR_COUNTER_MODE_CENTER_ALIGNED3:
            _freq = timebase_freq / float(cfg.hal_base_config.period * 2);
            break;
        }
    } else {
        fatal_error();
    }

    if (cfg.hal_base_config.period > UINT16_MAX) {
        fatal_error();
    }

    switch (config.hal_base_config.clockDivision) {
    case TMR_CLOCK_DIV_1:
        _t_dts_ns = float(config.hal_base_config.division + 1) * 1000000000.f /
                    float(core_clk_freq());
        break;
    case TMR_CLOCK_DIV_2:
        _t_dts_ns = 2 * float(config.hal_base_config.division + 1) *
                    1000000000.f / float(core_clk_freq());
        break;
    case TMR_CLOCK_DIV_4:
        _t_dts_ns = 4 * float(config.hal_base_config.division + 1) *
                    1000000000.f / float(core_clk_freq());
        break;
    }

    if (config.arr_preload) {
        _reg->CTRL1_B.ARPEN = 1;
    }

    TMR_ConfigTimeBase(_reg, &cfg.hal_base_config);

    _init_bdt(config, pin_bkin);
}

void PwmTimer::_init_bdt(const PwmConfig& config, BkinPin* pin_bkin) {
    if (config.hal_bdt_config.BRKState == TMR_BRK_STATE_ENABLE &&
        pin_bkin == nullptr) {
        fatal_error();
    }

    auto bdt_cfg = config.hal_bdt_config;

    if (bdt_cfg.deadTime == 0) {
        // deadtime specified by deadtime_ns
        _deadtime = config.deadtime_ns * 1E-09f;
        if (config.deadtime_ns <= 0X7F * _t_dts_ns) {
            bdt_cfg.deadTime = uint16_t(config.deadtime_ns / _t_dts_ns);
        } else if (config.deadtime_ns <= 127 * 2 * _t_dts_ns) {
            bdt_cfg.deadTime =
                    uint16_t((config.deadtime_ns - 64 * 2 * _t_dts_ns) /
                             (2 * _t_dts_ns));
            bdt_cfg.deadTime |= 0x80;
        } else if (config.deadtime_ns <= 63 * 8 * _t_dts_ns) {
            bdt_cfg.deadTime =
                    uint16_t((config.deadtime_ns - 32 * 8 * _t_dts_ns) /
                             (8 * _t_dts_ns));
            bdt_cfg.deadTime |= 0xC0;
        } else if (config.deadtime_ns <= 63 * 16 * _t_dts_ns) {
            bdt_cfg.deadTime =
                    uint16_t((config.deadtime_ns - 32 * 16 * _t_dts_ns) /
                             (16 * _t_dts_ns));
            bdt_cfg.deadTime |= 0xE0;
        } else {
            fatal_error();
        }
    } else {
        auto dtg = bdt_cfg.deadTime;
        if ((dtg & 0x80) == 0) {
            _deadtime = float(dtg) * _t_dts_ns * 1E-09f;
        } else if ((dtg & 0xC0) == 0x80) {
            _deadtime = float(64 + (dtg & 0x3F)) * 2 * _t_dts_ns * 1E-09f;
        } else if ((dtg & 0xE0) == 0xC0) {
            _deadtime = float(32 + (dtg & 0x1F)) * 8 * _t_dts_ns * 1E-09f;
        } else if ((dtg & 0xE0) == 0xE0) {
            _deadtime = float(32 + (dtg & 0x1F)) * 16 * _t_dts_ns * 1E-09f;
        } else {
            fatal_error();
        }
    }

    TMR_ConfigBDT(_reg, &bdt_cfg);
}

void PwmTimer::init_channel(Channel channel,
                            ChPin* pin_ch,
                            ChPin* pin_chn,
                            const PwmChannelConfig& config) {
    auto cfg = config;

    switch (channel) {
    case Channel::channel1:
        _reg->CCM1_COMPARE_B.OC1PEN = config.oc_preload & 0x01;
        TMR_ConfigOC1(_reg, &cfg.hal_oc_config);
        if (pin_ch) {
            _reg->CCEN_B.CC1EN = 1;
        } else {
            _reg->CCEN_B.CC1EN = 0;
        }
        if (pin_chn) {
            _reg->CCEN_B.CC1NEN = 1;
        } else {
            _reg->CCEN_B.CC1NEN = 0;
        }
        break;
    case Channel::channel2:
        _reg->CCM1_COMPARE_B.OC2PEN = config.oc_preload & 0x01;
        TMR_ConfigOC2(_reg, &cfg.hal_oc_config);
        if (pin_ch) {
            _reg->CCEN_B.CC2EN = 1;
        } else {
            _reg->CCEN_B.CC2EN = 0;
        }
        if (pin_chn) {
            _reg->CCEN_B.CC2NEN = 1;
        } else {
            _reg->CCEN_B.CC2NEN = 0;
        }
        break;
    case Channel::channel3:
        _reg->CCM2_COMPARE_B.OC3PEN = config.oc_preload & 0x01;
        TMR_ConfigOC3(_reg, &cfg.hal_oc_config);
        if (pin_ch) {
            _reg->CCEN_B.CC3EN = 1;
        } else {
            _reg->CCEN_B.CC3EN = 0;
        }
        if (pin_chn) {
            _reg->CCEN_B.CC3NEN = 1;
        } else {
            _reg->CCEN_B.CC3NEN = 0;
        }
        break;
    case Channel::channel4:
        _reg->CCM2_COMPARE_B.OC4PEN = config.oc_preload & 0x01;
        TMR_ConfigOC4(_reg, &cfg.hal_oc_config);
        if (pin_ch) {
            _reg->CCEN_B.CC4EN = 1;
        } else {
            _reg->CCEN_B.CC4EN = 0;
        }
        break;
    }
}

void PwmTimer::init_update_interrupts(IrqPriority priority) {
    _reg->DIEN_B.UIEN = 1;
    set_irq_priority(impl::up_irq_nums[std::to_underlying(_peripheral)],
                     priority);
}

void PwmTimer::init_break_interrupts(IrqPriority priority) {
    _reg->DIEN_B.BRKIEN = 1;
    set_irq_priority(impl::brk_irq_nums[std::to_underlying(_peripheral)],
                     priority);
    _brk_enabled = true;
}

} // namespace adv
} // namespace tim
} // namespace apm32
} // namespace mcu

#endif
#endif
