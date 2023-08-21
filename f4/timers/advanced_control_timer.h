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


enum class Channel {
    channel1 = TIM_CHANNEL_1,
    channel2 = TIM_CHANNEL_2,
    channel3 = TIM_CHANNEL_3,
    channel4 = TIM_CHANNEL_4,
};

struct ChannelConfig {
    TIM_OC_InitTypeDef hal_init;
};


struct PwmPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
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

    void init_pwm_channel(Channel channel, ChannelConfig config, const PwmPinConfig& pin_config);

    void start_pwm(Channel channel) {
        if (HAL_TIM_PWM_Start(&_handle, std::to_underlying(channel)) != HAL_OK) {
            fatal_error("timer pwm channel start failed");
        }
    }

    void stop_pwm(Channel channel) {
        if (HAL_TIM_PWM_Stop(&_handle, std::to_underlying(channel)) != HAL_OK) {
            fatal_error("timer pwm channel start failed");
        }
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
