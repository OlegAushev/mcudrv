#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include "timerdef.h"
#include <utility>


namespace mcu {


namespace timers {


namespace adv {


enum class Peripheral : unsigned int {
    tim1,
    tim8
};


constexpr size_t peripheral_count = 2;


namespace impl {


inline const std::array<TIM_TypeDef*, peripheral_count> instances = {TIM1, TIM8};


inline Peripheral to_peripheral(const TIM_TypeDef* instance) {
    return static_cast<Peripheral>(std::distance(instances.begin(),
                                   std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ __HAL_RCC_TIM1_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM8_CLK_ENABLE(); },
};


inline constexpr std::array<IRQn_Type, peripheral_count> up_irq_nums = {TIM1_UP_TIM10_IRQn, TIM8_UP_TIM13_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> brk_irq_nums = {TIM1_BRK_TIM9_IRQn, TIM8_BRK_TIM12_IRQn};


} // namespace impl


class Timer : public emb::interrupt_invoker_array<Timer, peripheral_count>, public emb::noncopyable {
private:
    const Peripheral _peripheral;
    TIM_HandleTypeDef _handle{};
    TIM_TypeDef* _reg;

    static inline std::array<bool, peripheral_count> _clk_enabled{};

    float _freq{0};
    float _t_dts_ns{0};
    float _deadtime{0};

    bool _brk_enabled{false};
public:
    Timer(Peripheral peripheral, const PwmConfig& config);
    Peripheral peripheral() const { return _peripheral; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    TIM_TypeDef* reg() { return _reg; }
    static Timer* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Timer, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void init_pwm(Channel channel, ChPin* pin_ch, ChPin* pin_chn, ChannelConfig config);
    void init_bdt(BkinPin* pin_bkin, BdtConfig config);

    void start() {
        set_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }

    void stop() {
        clear_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }

    bool pwm_active() const {
        return bit_is_set<uint32_t>(_reg->BDTR, TIM_BDTR_MOE);
    }

    void start_pwm() {
        if (_brk_enabled) {
            clear_bit<uint32_t>(_reg->SR, TIM_SR_BIF);
            set_bit<uint32_t>(TIM1->DIER, TIM_DIER_BIE);
        }
        set_bit<uint32_t>(_reg->BDTR, TIM_BDTR_MOE);
    }

    void stop_pwm() {
        mcu::clear_bit<uint32_t>(_reg->BDTR, TIM_BDTR_MOE);
        if (_brk_enabled) {
            // disable break interrupts to prevent instant call of BRK ISR
            mcu::clear_bit<uint32_t>(TIM1->DIER, TIM_DIER_BIE);
        }
    }

    void set_duty_cycle(Channel channel, float duty_cycle) {
        uint32_t compare_value = static_cast<uint32_t>(duty_cycle * float(__HAL_TIM_GET_AUTORELOAD(&_handle)));
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

    void init_break_interrupts(IrqPriority priority);

    void enable_break_interrupts() {
        clear_bit<uint32_t>(_reg->SR, TIM_SR_BIF);
        clear_pending_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
    }
    
    void disable_break_interrupts() {
        disable_irq(impl::brk_irq_nums[std::to_underlying(_peripheral)]);
    }
private:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace adv


} // namespace timers


} // namepsace mcu


#endif
#endif
