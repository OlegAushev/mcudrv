#pragma once


#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib_stm32/core.h>
#include <utility>


namespace mcu {

namespace timers_gp {

enum class Peripheral {
    tim2,
    tim3,
    tim4,
    tim5
};


constexpr int peripheral_count = 4;


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

inline constexpr std::array<TIM_TypeDef*, peripheral_count> timer_instances = {TIM2, TIM3, TIM4, TIM5};


inline Peripheral to_peripheral(const TIM_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(timer_instances.begin(), std::find(timer_instances.begin(), timer_instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> timer_clk_enable_funcs = {
    [](){ __HAL_RCC_TIM2_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM3_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM4_CLK_ENABLE(); },
    [](){ __HAL_RCC_TIM5_CLK_ENABLE(); },	
};

}


class Timer : public emb::interrupt_invoker_array<Timer, peripheral_count>, emb::noncopyable {
private:
    const Peripheral _peripheral;
    TIM_HandleTypeDef _handle = {};

    static inline std::array<bool, peripheral_count> _clk_enabled = {};
public:
    Timer(Peripheral peripheral, const Config& config);
    Peripheral peripheral() const { return _peripheral; }
    TIM_HandleTypeDef* handle() { return &_handle; }
    static Timer* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Timer, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void init_pwm_channel(Channel channel, ChannelConfig config, const PwmPinConfig& pin_config) {
        mcu::gpio::Output({
            .port = pin_config.port, 
            .pin = {
                .Pin = pin_config.pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                .Alternate = pin_config.af_selection
            },
            .active_state = emb::gpio::ActiveState::high});

        if (HAL_TIM_PWM_ConfigChannel(&_handle, &config.hal_init, std::to_underlying(channel)) != HAL_OK) {
            fatal_error("timer pwm channel initialization failed");
        }
    }

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

protected:
    void enable_clk() {
        auto tim_idx = std::to_underlying(_peripheral);
        if (!_clk_enabled[tim_idx]) {
            impl::timer_clk_enable_funcs[tim_idx]();
            _clk_enabled[tim_idx] = true;
        }
    }
};













} // namespace timers

} // namespace mcu