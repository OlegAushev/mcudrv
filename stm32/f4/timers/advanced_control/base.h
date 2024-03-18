#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/timers/timersdef.h>


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


class AbstractTimer : public emb::interrupt_invoker_array<AbstractTimer, peripheral_count>, public emb::noncopyable {
protected:
    const Peripheral _peripheral;
    const OpMode _mode;
    TIM_HandleTypeDef _handle{};
    TIM_TypeDef* _reg;
    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    AbstractTimer(Peripheral peripheral, OpMode mode);
    OpMode mode() const { return _mode; }
    Peripheral peripheral() const { return _peripheral; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    TIM_TypeDef* reg() { return _reg; }

    void enable() {
        set_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }

    void disable() {
        clear_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }
private:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace impl


} // namespace adv


} // namespace timers


} // namepsace mcu


#endif
#endif
