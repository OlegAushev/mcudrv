#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/timdef.h>


namespace mcu {
namespace tim {    
namespace gen1 {


constexpr size_t peripheral_count = 2;
enum class Peripheral : unsigned int {
    tim2,
    tim5,
};


constexpr size_t channel_count = 4;
enum class Channel : unsigned int {
    channel1 = TIM_CHANNEL_1,
    channel2 = TIM_CHANNEL_2,
    channel3 = TIM_CHANNEL_3,
    channel4 = TIM_CHANNEL_4,
};


enum class InterruptSource : unsigned int {
    update  = TIM_IT_UPDATE,
    cc1     = TIM_IT_CC1,
    cc2     = TIM_IT_CC2,
    cc3     = TIM_IT_CC3,
    cc4     = TIM_IT_CC4,
    trigger = TIM_IT_TRIGGER
};


namespace impl {


inline const std::array<TIM_TypeDef*, peripheral_count> instances = {TIM2, TIM5};


inline Peripheral to_peripheral(const TIM_TypeDef* instance) {
    return static_cast<Peripheral>(std::distance(instances.begin(),
                                                 std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ __HAL_RCC_TIM2_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM5_CLK_ENABLE(); },
};


inline constexpr std::array<IRQn_Type, peripheral_count> irq_nums = {TIM2_IRQn, TIM5_IRQn};


class AbstractTimer : public emb::singleton_array<AbstractTimer, peripheral_count>, private emb::noncopyable {
protected:
    const Peripheral _peripheral;
    TIM_TypeDef* const _reg;
    const OpMode _mode;
    TIM_HandleTypeDef _handle{};
    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    AbstractTimer(Peripheral peripheral, OpMode mode)
            : emb::singleton_array<AbstractTimer, peripheral_count>(this, std::to_underlying(peripheral))
            , _peripheral(peripheral)
            , _reg(impl::instances[std::to_underlying(peripheral)])
            , _mode(mode)
    {
        _enable_clk(peripheral);
        _handle.Instance = _reg;
    }

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


} // namespace general
} // namespace timers
} // namepsace mcu


#endif
#endif
