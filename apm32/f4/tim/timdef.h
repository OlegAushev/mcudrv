#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/apm32f4_base.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <apm32f4xx_tmr.h>


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


struct ChPinConfig {
    GPIO_T* port;
    uint16_t pin;
    GPIO_AF_T af_selection;
};


class ChPin {
public:
    ChPin(const ChPinConfig& config) {
        mcu::gpio::AlternatePin({.port = config.port, 
                                 .pin = {.pin = config.pin,
                                         .mode = GPIO_MODE_AF,
                                         .speed = GPIO_SPEED_50MHz,
                                         .otype = GPIO_OTYPE_PP,
                                         .pupd = GPIO_PUPD_NOPULL},
                                 .af_selection = config.af_selection,
                                 .actstate = emb::gpio::active_pin_state::high});
    }
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
        mcu::gpio::AlternatePin({.port = config.port, 
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
