#pragma once

#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"


namespace mcu {

namespace timers {


struct Config {
    uint32_t freq;
    TIM_Base_InitTypeDef hal_base_cfg;
};


enum class Channel {
    channel1 = TIM_CHANNEL_1,
    channel2 = TIM_CHANNEL_2,
    channel3 = TIM_CHANNEL_3,
    channel4 = TIM_CHANNEL_4,
};


struct ChannelConfig {
    TIM_OC_InitTypeDef hal_oc_cfg;
};


struct ChPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


class ChPin {
public:
    ChPin(const ChPinConfig& config) {
        mcu::gpio::Output({
            .port = config.port, 
            .pin = {
                .Pin = config.pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_HIGH,
                .Alternate = config.af_selection
            },
            .active_state = emb::gpio::ActiveState::high});
    }
};


struct BdtConfig {
    uint32_t deadtime_ns;
    TIM_BreakDeadTimeConfigTypeDef hal_bdt_cfg;
};


struct BkinPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t pull;
    uint32_t af_selection;
};


class BkinPin {
public:
    BkinPin(const BkinPinConfig& config) {
        mcu::gpio::Input({
            .port = config.port, 
            .pin = {
                .Pin = config.pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = config.pull,
                .Speed = GPIO_SPEED_FREQ_HIGH,
                .Alternate = config.af_selection
            },
            .active_state = emb::gpio::ActiveState::high});
    }
};


} // namespace timers

} // namepsace mcu

#endif
