#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "../apm32f4_common.h"
#include "../gpio/gpio.h"
#include "../system/system.h"
#include <apm32f4xx_tmr.h>


namespace mcu {


namespace timers {


struct Config {
    float freq;
    TMR_BaseConfig_T hal_base_config;
    bool arr_preload;
};


enum class Channel : unsigned int {
    channel1 = TMR_CHANNEL_1,
    channel2 = TMR_CHANNEL_2,
    channel3 = TMR_CHANNEL_3,
    channel4 = TMR_CHANNEL_4,
};


struct ChannelConfig {
    TMR_OCConfig_T hal_oc_config;
    TMR_OC_PRELOAD_T oc_preload;
};


struct ChPinConfig {
    GPIO_T* port;
    uint16_t pin;
    GPIO_AF_T af_selection;
};


class ChPin {
public:
    ChPin(const ChPinConfig& config) {
        mcu::gpio::AlternateIO({.port = config.port, 
                                .pin = {.pin = config.pin,
                                        .mode = GPIO_MODE_AF,
                                        .speed = GPIO_SPEED_50MHz,
                                        .otype = GPIO_OTYPE_PP,
                                        .pupd = GPIO_PUPD_NOPULL},
                                .af_selection = config.af_selection,
                                .actstate = emb::gpio::active_pin_state::high});
    }
};


struct BdtConfig {
    float deadtime_ns;
    TMR_BDTConfig_T hal_bdt_config;
};


struct BkinPinConfig {
    GPIO_T* port;
    uint16_t pin;
    GPIO_PUPD_T pull;
    GPIO_AF_T af_selection;
};


class BkinPin {
public:
    BkinPin(const BkinPinConfig& config) {
        mcu::gpio::AlternateIO({.port = config.port, 
                                .pin = {.pin = config.pin,
                                        .mode = GPIO_MODE_AF,
                                        .speed = GPIO_SPEED_50MHz,
                                        .otype = GPIO_OTYPE_PP,
                                        .pupd = config.pull},
                                .af_selection = config.af_selection,
                                .actstate = emb::gpio::active_pin_state::high});
    }
};


} // namespace timers


} // namepsace mcu


#endif
#endif
