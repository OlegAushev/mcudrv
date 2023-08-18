#pragma once

#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <utility>


namespace mcu {

namespace timers {


enum class AdvancedControlPeripheral {
    tim1,
    tim8
};


constexpr int adv_peripheral_count = 2;


struct Config {
    uint32_t freq;
    TIM_Base_InitTypeDef hal_init;
};


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
public:
    AdvancedControlTimer(AdvancedControlPeripheral peripheral, const Config& config);
    AdvancedControlPeripheral peripheral() const { return _peripheral; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    static AdvancedControlTimer* instance(AdvancedControlPeripheral peripheral) {
        return emb::interrupt_invoker_array<AdvancedControlTimer, adv_peripheral_count>::instance(std::to_underlying(peripheral));
    }
private:
    void _enable_clk();
};


} // namespace timers

} // namepsace mcu

#endif
