#pragma once

#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"


namespace mcu {

namespace timers {


struct Config {
    uint32_t freq;
    TIM_Base_InitTypeDef hal_init;
    TIM_BreakDeadTimeConfigTypeDef bdt_hal_init;
};


enum class Channel {
    channel1 = TIM_CHANNEL_1,
    channel2 = TIM_CHANNEL_2,
    channel3 = TIM_CHANNEL_3,
    channel4 = TIM_CHANNEL_4,
};


struct ChannelConfig {
    TIM_OC_InitTypeDef oc_hal_init;
};


struct PinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


class ChPin {
public:
    ChPin(const PinConfig& config) {
        mcu::gpio::Output({
            .port = config.port, 
            .pin = {
                .Pin = config.pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
                .Alternate = config.af_selection
            },
            .active_state = emb::gpio::ActiveState::high});
    }
};


} // namespace timers

} // namepsace mcu

#endif
