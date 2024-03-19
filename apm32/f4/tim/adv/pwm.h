#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/tim/adv/base.h>


namespace mcu {


namespace tim {


namespace adv {


struct PwmConfig {
    float freq;
    float deadtime_ns;
    bool arr_preload;
    TMR_BaseConfig_T hal_base_config;
    TMR_BDTConfig_T hal_bdt_config;
};


class PwmTimer : public impl::AbstractTimer {
private:
    float _freq{0};
    float _t_dts_ns{0};
    float _deadtime{0};
    bool _brk_enabled{false};
public:
    PwmTimer(Peripheral peripheral, const PwmConfig& config, BkinPin* pin_bkin);

    static PwmTimer* instance(Peripheral peripheral) {
        assert(impl::AbstractTimer::instance(std::to_underlying(peripheral))->mode() == OpMode::pwm_generation);
        return static_cast<PwmTimer*>(impl::AbstractTimer::instance(std::to_underlying(peripheral)));
    }

    void initialize_channel(Channel channel, ChPin* pin_ch, ChPin* pin_chn, const ChannelConfig& config);
    
    bool active() const {
        return _reg->BDT_B.MOEN == 1;
    }

    void start() {
        if (_brk_enabled) {
            _reg->STS_B.BRKIFLG = 0;
            _reg->DIEN_B.BRKIEN = 1;
        }
        _reg->BDT_B.MOEN = 1;
    }

    void stop() {
        _reg->BDT_B.MOEN = 0;
        if (_brk_enabled) {
            // disable break interrupts to prevent instant call of BRK ISR
            _reg->DIEN_B.BRKIEN = 0;
        }
    }

    void set_duty_cycle(Channel channel, float duty_cycle) {
        uint32_t compare_value = static_cast<uint32_t>(duty_cycle * float(_reg->AUTORLD));
        switch (channel) {
        case Channel::channel1:
            write_reg(_reg->CC1, compare_value); 
            break;
        case Channel::channel2:
            write_reg(_reg->CC2, compare_value); 
            break;
        case Channel::channel3:
            write_reg(_reg->CC3, compare_value); 
            break;
        case Channel::channel4:
            write_reg(_reg->CC4, compare_value); 
            break;
        }
    }

    float freq() const { return _freq; }

    void initialize_update_interrupts(IrqPriority priority);

    void enable_update_interrupts() {
        _reg->STS_B.UIFLG = 0;
        clear_pending_irq(impl::up_irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::up_irq_nums[std::to_underlying(_peripheral)]);
    }

    void disable_update_interrupts() {
        disable_irq(impl::up_irq_nums[std::to_underlying(_peripheral)]);
    }

    void initialize_break_interrupts(IrqPriority priority);

    void enable_break_interrupts() {
        _reg->STS_B.BRKIFLG = 0;
        clear_pending_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
    }
    
    void disable_break_interrupts() {
        disable_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
    }
private:
    void _initialize_bdt(const PwmConfig& config, BkinPin* pin_bkin);
};


} // namespace adv


} // namespace timers


} // namepsace mcu


#endif
#endif
