#pragma once

#ifdef STM32F4xx

#include "timers_def.h"
#include <utility>


namespace mcu {

namespace timers {


enum class AdvancedControlPeripheral {
    tim1,
    tim8
};


constexpr int adv_peripheral_count = 2;


namespace impl {

inline constexpr std::array<TIM_TypeDef*, adv_peripheral_count> adv_timer_instances = {TIM1, TIM8};


inline AdvancedControlPeripheral to_peripheral(const TIM_TypeDef* instance) {
    return static_cast<AdvancedControlPeripheral>(
        std::distance(adv_timer_instances.begin(),
                      std::find(adv_timer_instances.begin(),
                                adv_timer_instances.end(),
                                instance)));
}


inline std::array<void(*)(void), adv_peripheral_count> adv_timer_clk_enable_funcs = {
    [](){ __HAL_RCC_TIM1_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM8_CLK_ENABLE(); },
};

}


class AdvancedControlTimer : public emb::interrupt_invoker_array<AdvancedControlTimer, adv_peripheral_count>, public emb::noncopyable {
private:
    const AdvancedControlPeripheral _peripheral;
    TIM_HandleTypeDef _handle = {};

    static inline std::array<bool, adv_peripheral_count> _clk_enabled = {};

    uint32_t _freq = 0;
    float _t_dts_ns = 0;
public:
    AdvancedControlTimer(AdvancedControlPeripheral peripheral, const Config& config);
    AdvancedControlPeripheral peripheral() const { return _peripheral; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    static AdvancedControlTimer* instance(AdvancedControlPeripheral peripheral) {
        return emb::interrupt_invoker_array<AdvancedControlTimer, adv_peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void init_pwm(Channel channel, ChannelConfig config, ChPin* pin_ch, ChPin* pin_chn);
    void init_bdt(BdtConfig config, BkinPin* pin_bkin);

    void start_pwm() {
        mcu::set_bit(_handle.Instance->BDTR, TIM_BDTR_MOE);
    }

    void stop_pwm() {
        mcu::clear_bit(_handle.Instance->BDTR, TIM_BDTR_MOE);
    }

    void set_duty_cycle(Channel channel, float duty_cycle) {
        uint32_t compare_value = static_cast<uint32_t>(duty_cycle * float(__HAL_TIM_GET_AUTORELOAD(&_handle)));
        __HAL_TIM_SET_COMPARE(&_handle, std::to_underlying(channel), compare_value);
    }
private:
    void _enable_clk();
};


} // namespace timers

} // namepsace mcu

#endif
