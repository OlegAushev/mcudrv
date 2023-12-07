#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "timers_common.h"


namespace mcu {


namespace timers {


constexpr size_t adv_timer_peripheral_count = 2;
enum class AdvancedControlPeripheral : unsigned int {
    tim1,
    tim8
};


namespace impl {


inline const std::array<TMR_T*, adv_timer_peripheral_count> adv_timer_instances = {TMR1, TMR8};


inline AdvancedControlPeripheral to_peripheral(const TMR_T* instance) {
    return static_cast<AdvancedControlPeripheral>(
        std::distance(adv_timer_instances.begin(),
                      std::find(adv_timer_instances.begin(),
                                adv_timer_instances.end(),
                                instance)));
}


inline std::array<void(*)(void), adv_timer_peripheral_count> adv_timer_clk_enable_funcs = {
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR1); },
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR8); },
};


inline constexpr std::array<IRQn_Type, adv_timer_peripheral_count> adv_timer_up_irqn = {TMR1_UP_TMR10_IRQn, TMR8_UP_TMR13_IRQn};
inline constexpr std::array<IRQn_Type, adv_timer_peripheral_count> adv_timer_brk_irqn = {TMR1_BRK_TMR9_IRQn, TMR8_BRK_TMR12_IRQn};


} // namespace impl


class AdvancedControlTimer : public emb::interrupt_invoker_array<AdvancedControlTimer, adv_timer_peripheral_count>, public emb::noncopyable {
private:
    const AdvancedControlPeripheral _peripheral;
    TMR_T* _reg;

    static inline std::array<bool, adv_timer_peripheral_count> _clk_enabled{};

    float _freq{0};
    float _t_dts_ns{0};

    bool _brk_enabled{false};
public:
    AdvancedControlTimer(AdvancedControlPeripheral peripheral, Config config);
    AdvancedControlPeripheral peripheral() const { return _peripheral; }
    TMR_T* reg() { return _reg; }
    static AdvancedControlTimer* instance(AdvancedControlPeripheral peripheral) {
        return emb::interrupt_invoker_array<AdvancedControlTimer, adv_timer_peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void init_pwm(Channel channel, ChPin* pin_ch, ChPin* pin_chn, ChannelConfig config);
    void init_bdt(BkinPin* pin_bkin, BdtConfig config);

    void start() {
        _reg->CTRL1_B.CNTEN = 1;
    }

    void stop() {
        _reg->CTRL1_B.CNTEN = 0;
    }

    bool pwm_active() const {
        return _reg->BDT_B.MOEN == 1;
    }

    void start_pwm() {
        if (_brk_enabled) {
            _reg->STS_B.BRKIFLG = 0;
            _reg->DIEN_B.BRKIEN = 1;
        }
        _reg->BDT_B.MOEN = 1;
    }

    void stop_pwm() {
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

    void init_update_interrupts(IrqPriority priority);

    void enable_update_interrupts() {
        _reg->STS_B.UIFLG = 0;
        clear_pending_irq(impl::adv_timer_up_irqn[std::to_underlying(_peripheral)]);
        enable_irq(impl::adv_timer_up_irqn[std::to_underlying(_peripheral)]);
    }

    void disable_update_interrupts() {
        disable_irq(impl::adv_timer_up_irqn[std::to_underlying(_peripheral)]);
    }

    void init_break_interrupts(IrqPriority priority);

    void enable_break_interrupts() {
        _reg->STS_B.BRKIFLG = 0;
        clear_pending_irq(impl::adv_timer_brk_irqn[std::to_underlying(_peripheral)]);
        enable_irq(impl::adv_timer_brk_irqn[std::to_underlying(_peripheral)]);
    }
    
    void disable_break_interrupts() {
        disable_irq(impl::adv_timer_brk_irqn[std::to_underlying(_peripheral)]);
    }
private:
    static void _enable_clk(AdvancedControlPeripheral peripheral);
};


} // namespace timers


} // namepsace mcu


#endif
#endif
