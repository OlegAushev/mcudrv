#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/timdef.h>


namespace mcu {
namespace tim {
namespace adv {


constexpr size_t peripheral_count = 2;
enum class Peripheral : unsigned int {
    tim1,
    tim8
};


constexpr size_t channel_count = 4;
enum class Channel : unsigned int {
    channel1 = TIM_CHANNEL_1,
    channel2 = TIM_CHANNEL_2,
    channel3 = TIM_CHANNEL_3,
    channel4 = TIM_CHANNEL_4,
};


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
    TIM_TypeDef* const _reg;
    const OpMode _mode;
    TIM_HandleTypeDef _handle{};
    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    AbstractTimer(Peripheral peripheral, OpMode mode)
            : emb::interrupt_invoker_array<AbstractTimer, peripheral_count>(this, std::to_underlying(peripheral))
            , _peripheral(peripheral)
            , _reg(impl::instances[std::to_underlying(peripheral)])
            , _mode(mode) {
        _enable_clk(peripheral);
        _handle.Instance = _reg;
    }

    Peripheral peripheral() const { return _peripheral; }
    TIM_TypeDef* reg() { return _reg; }
    OpMode mode() const { return _mode; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    
    CountDir dir() const {
        if (bit_is_set<uint32_t>(_reg->CR1, TIM_CR1_DIR)) {
            return CountDir::down;
        }
        return CountDir::up;
    }

    void enable() {
        set_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }

    void disable() {
        clear_bit<uint32_t>(_reg->CR1, TIM_CR1_CEN);
    }
private:
    static void _enable_clk(Peripheral peripheral)  {
        auto timer_idx = std::to_underlying(peripheral);
        if (_clk_enabled[timer_idx]) {
            return;
        }

        impl::clk_enable_funcs[timer_idx]();
        _clk_enabled[timer_idx] = true;
    }
};


} // namespace impl


} // namespace adv
} // namespace timers
} // namepsace mcu


#endif
#endif
