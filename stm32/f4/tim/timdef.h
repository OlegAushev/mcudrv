#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/stm32_f4_base.h>
#include <mcudrv/stm32/f4/system/system.h>
#include <mcudrv/stm32/f4/gpio/gpio.h>


namespace mcu {


namespace tim {


enum class OpMode {
    inactive,
    timebase,
    input_capture,
    output_compare,
    pwm_generation,
    one_pulse
};


struct InputCaptureConfig {
    TIM_Base_InitTypeDef hal_base_config;
};


struct ChannelConfig {
    TIM_OC_InitTypeDef hal_oc_config;
};


struct ChPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


class ChPin {
public:
    ChPin(const ChPinConfig& config) {
        mcu::gpio::AlternatePin({
            .port = config.port, 
            .pin = {
                .Pin = config.pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_HIGH,
                .Alternate = config.af_selection
            },
            .actstate = emb::gpio::active_pin_state::high});
    }
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
        mcu::gpio::AlternatePin({
            .port = config.port, 
            .pin = {
                .Pin = config.pin,
                .Mode = GPIO_MODE_AF_PP,
                .Pull = config.pull,
                .Speed = GPIO_SPEED_FREQ_HIGH,
                .Alternate = config.af_selection
            },
            .actstate = emb::gpio::active_pin_state::high});
    }
};


} // namespace timers


} // namepsace mcu


#endif
#endif
