#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/adv/base.h>
#include <emblib/math.hpp>


namespace mcu {
namespace tim {
namespace adv {


struct PwmConfig {
    float freq;
    float deadtime_ns;
    TIM_Base_InitTypeDef hal_base_config;
    TIM_BreakDeadTimeConfigTypeDef hal_bdt_config;
};


struct PwmChannelConfig {
    TIM_OC_InitTypeDef hal_oc_config;
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

    void init_channel(Channel channel, ChPin* pin_ch, ChPin* pin_chn, PwmChannelConfig config);

    bool active() const {
        return bit_is_set<uint32_t>(_reg->BDTR, TIM_BDTR_MOE);
    }

    void start() {
        if (_brk_enabled) {
            clear_bit<uint32_t>(_reg->SR, TIM_SR_BIF);
            set_bit<uint32_t>(TIM1->DIER, TIM_DIER_BIE);
        }
        set_bit<uint32_t>(_reg->BDTR, TIM_BDTR_MOE);
    }

    void stop() {
        mcu::clear_bit<uint32_t>(_reg->BDTR, TIM_BDTR_MOE);
        if (_brk_enabled) {
            // disable break interrupts to prevent instant call of BRK ISR
            mcu::clear_bit<uint32_t>(TIM1->DIER, TIM_DIER_BIE);
        }
    }

    emb::unsigned_perunit duty_cycle(Channel channel) const {
        switch (channel) {
        case Channel::channel1:
            return emb::unsigned_perunit(float(_reg->CCR1) / float(_reg->ARR)); 
        case Channel::channel2:
            return emb::unsigned_perunit(float(_reg->CCR2) / float(_reg->ARR)); 
        case Channel::channel3:
            return emb::unsigned_perunit(float(_reg->CCR3) / float(_reg->ARR)); 
        case Channel::channel4:
            return emb::unsigned_perunit(float(_reg->CCR4) / float(_reg->ARR)); 
        }
        return {};
    }

    void set_duty_cycle(Channel channel, emb::unsigned_perunit duty_cycle) {
        uint32_t compare_value = uint32_t(duty_cycle.get() * float(_reg->ARR));
        switch (channel) {
        case Channel::channel1:
            write_reg(_reg->CCR1, compare_value); 
            break;
        case Channel::channel2:
            write_reg(_reg->CCR2, compare_value); 
            break;
        case Channel::channel3:
            write_reg(_reg->CCR3, compare_value); 
            break;
        case Channel::channel4:
            write_reg(_reg->CCR4, compare_value); 
            break;
        }
    }

    float freq() const { return _freq; }
    float deadtime() const { return _deadtime; }

    void init_update_interrupts(IrqPriority priority);

    void enable_update_interrupts() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_UIF);
        clear_pending_irq(impl::up_irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::up_irq_nums[std::to_underlying(_peripheral)]);
    }

    void disable_update_interrupts() {
        disable_irq(impl::up_irq_nums[std::to_underlying(_peripheral)]);
    }

    void ack_update_interrupt() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_UIF);
    }

    void init_break_interrupts(IrqPriority priority);

    void enable_break_interrupts() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_BIF);
        clear_pending_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
    }
    
    void disable_break_interrupts() {
        disable_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
    }

    void ack_break_interrupt() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_BIF);
    }
private:
    void _init_bdt(const PwmConfig& bdt_config, BkinPin* pin_bkin);
};


} // namespace adv
} // namespace timers
} // namepsace mcu


#endif
#endif
