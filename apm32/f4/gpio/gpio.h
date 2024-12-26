#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_gpio.h>
#include <apm32f4xx_rcm.h>

#include <mcudrv/apm32/f4/apm32f4_base.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/generic/gpio.hpp>

#include <algorithm>
#include <array>
#include <optional>
#include <utility>

namespace mcu {
namespace apm32 {
namespace gpio {

constexpr size_t port_count = 9;

enum class Port : unsigned int {
    gpioa,
    gpiob,
    gpioc,
    gpiod,
    gpioe,
    gpiof,
    gpiog,
    gpioh,
    gpioi
};

enum class Pin : uint16_t {
    pin0 = GPIO_PIN_0,
    pin1 = GPIO_PIN_1,
    pin2 = GPIO_PIN_2,
    pin3 = GPIO_PIN_3,
    pin4 = GPIO_PIN_4,
    pin5 = GPIO_PIN_5,
    pin6 = GPIO_PIN_6,
    pin7 = GPIO_PIN_7,
    pin8 = GPIO_PIN_8,
    pin9 = GPIO_PIN_9,
    pin10 = GPIO_PIN_10,
    pin11 = GPIO_PIN_11,
    pin12 = GPIO_PIN_12,
    pin13 = GPIO_PIN_13,
    pin14 = GPIO_PIN_14,
    pin15 = GPIO_PIN_15,
};

struct PinConfig {
    Port port;
    Pin pin;
    GPIO_Config_T config;
    GPIO_AF_T altfunc;
    mcu::gpio::active_state active_state;
};

namespace impl {

inline const std::array<GPIO_T*, port_count> gpio_instances = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI};

inline std::array<void (*)(void), port_count> gpio_clk_enable_funcs = {
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOA); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOB); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOC); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOD); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOE); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOF); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOG); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOH); },
    []() { RCM_EnableAHB1PeriphClock(RCM_AHB1_PERIPH_GPIOI); }};

class GpioPin {
private:
    static inline std::array<uint16_t, port_count> _assigned{};
    static inline std::array<bool, port_count> _clk_enabled{};
protected:
    // Config _cfg;
    bool _initialized{false};
    GPIO_T* _port;
    uint16_t _pin;
    std::optional<mcu::gpio::active_state> _active_state{std::nullopt};
    GpioPin() = default;
public:
    void init(PinConfig config) {
        const size_t port_idx = std::to_underlying(config.port);

        _port = impl::gpio_instances[port_idx];
        _pin = std::to_underlying(config.pin);
        _active_state = config.active_state;
        config.config.pin = _pin;

        if (_assigned[port_idx] & _pin) {
            fatal_error();
        }
        _assigned[port_idx] |= uint16_t(_pin);

        if (!_clk_enabled[port_idx]) {
            gpio_clk_enable_funcs[port_idx]();
            _clk_enabled[port_idx] = true;
        }

        if (config.config.mode == GPIO_MODE_AF) {
            GPIO_ConfigPinAF(_port,
                             static_cast<GPIO_PIN_SOURCE_T>(bit_position(_pin)),
                             config.altfunc);
        }

        GPIO_Config(_port, &config.config);
        _initialized = true;
    }

    unsigned int pin_no() const { return __CLZ(__RBIT(_pin)); }
    uint16_t pin_bit() const { return static_cast<uint16_t>(_pin); }
    const GPIO_T* port() const { return _port; }
    bool initialized() const { return _initialized; }
};

} // namespace impl

class InputPin : public mcu::gpio::input_pin, public impl::GpioPin {
    // friend void ::EXTI0_IRQHandler();
    // friend void ::EXTI1_IRQHandler();
    // friend void ::EXTI2_IRQHandler();
    // friend void ::EXTI3_IRQHandler();
    // friend void ::EXTI4_IRQHandler();
    // friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
public:
    InputPin() = default;
    InputPin(const PinConfig& config) {
        if (config.config.mode != GPIO_MODE_IN) {
            fatal_error();
        }
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert(_initialized);
        if ((read_reg(_port->IDATA) & _pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual mcu::gpio::pin_state read() const override {
        assert(_initialized);
        if (read_level() == std::to_underlying(*_active_state)) {
            return mcu::gpio::pin_state::active;
        }
        return mcu::gpio::pin_state::inactive;
    }
    // TODO
    // private:
    //     IRQn_Type _irqn = NonMaskableInt_IRQn;	// use NonMaskableInt_IRQn as value for not initialized interrupt
    //     static inline std::array<void(*)(void), 16> on_interrupt = {
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //         emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    //     };
    // public:
    //     void initialize_interrupt(void(*handler)(void), IrqPriority priority) {
    //         switch (_config.pin.Pin) {
    //         case GPIO_PIN_0:
    //             _irqn = EXTI0_IRQn;
    //             break;
    //         case GPIO_PIN_1:
    //             _irqn = EXTI1_IRQn;
    //             break;
    //         case GPIO_PIN_2:
    //             _irqn = EXTI2_IRQn;
    //             break;
    //         case GPIO_PIN_3:
    //             _irqn = EXTI3_IRQn;
    //             break;
    //         case GPIO_PIN_4:
    //             _irqn = EXTI4_IRQn;
    //             break;
    //         case GPIO_PIN_5: case GPIO_PIN_6: case GPIO_PIN_7: case GPIO_PIN_8: case GPIO_PIN_9:
    //             _irqn = EXTI9_5_IRQn;
    //             break;
    //         case GPIO_PIN_10: case GPIO_PIN_11: case GPIO_PIN_12: case GPIO_PIN_13: case GPIO_PIN_14: case GPIO_PIN_15:
    //             _irqn = EXTI15_10_IRQn;
    //             break;
    //         default:
    //             _irqn = NonMaskableInt_IRQn;
    //             return;
    //         }
    //         HAL_NVIC_SetPriority(_irqn, priority.get(), 0);
    //         on_interrupt[this->pin_no()] = handler;
    //     }

    //     void enable_interrupts() {
    //         if (_irqn != NonMaskableInt_IRQn) {
    //             HAL_NVIC_EnableIRQ(_irqn);
    //         }
    //     }

    //     void disable_interrupts() {
    //         if (_irqn != NonMaskableInt_IRQn) {
    //             HAL_NVIC_EnableIRQ(_irqn);
    //         }
    //     }
};

class OutputPin : public mcu::gpio::output_pin, public impl::GpioPin {
public:
    OutputPin() = default;
    OutputPin(
            const PinConfig& config,
            mcu::gpio::pin_state init_state = mcu::gpio::pin_state::inactive) {
        if (config.config.mode != GPIO_MODE_OUT) {
            fatal_error();
        }
        init(config);
        OutputPin::set(init_state);
    }

    virtual unsigned int read_level() const override {
        assert(_initialized);
        if ((read_reg(_port->IDATA) & _pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual void set_level(unsigned int level) override {
        assert(_initialized);
        if (level != 0) {
            write_reg(_port->BSCL, _pin);
        } else {
            write_reg(_port->BSCH, _pin);
        }
    }

    virtual mcu::gpio::pin_state read() const override {
        assert(_initialized);
        if (read_level() == std::to_underlying(*_active_state)) {
            return mcu::gpio::pin_state::active;
        }
        return mcu::gpio::pin_state::inactive;
    }

    virtual void
    set(mcu::gpio::pin_state st = mcu::gpio::pin_state::active) override {
        assert(_initialized);
        if (st == mcu::gpio::pin_state::active) {
            set_level(std::to_underlying(*_active_state));
        } else {
            set_level(1 - std::to_underlying(*_active_state));
        }
    }

    virtual void reset() override {
        assert(_initialized);
        set(mcu::gpio::pin_state::inactive);
    }

    virtual void toggle() override {
        assert(_initialized);
        uint16_t odr_reg = static_cast<uint16_t>(read_reg(_port->ODATA));
        write_reg<uint16_t>(_port->BSCL, ~odr_reg & _pin);
        write_reg<uint16_t>(_port->BSCH, odr_reg & _pin);
    }
};

class AlternatePin : public impl::GpioPin {
public:
    AlternatePin() = default;
    AlternatePin(const PinConfig& config) {
        if (config.config.mode != GPIO_MODE_AF) {
            fatal_error();
        }
        init(config);
    }
};

class AnalogPin : public impl::GpioPin {
public:
    AnalogPin() = default;
    AnalogPin(const PinConfig& config) {
        if (config.config.mode != GPIO_MODE_AN) {
            fatal_error();
        }
        init(config);
    }
};

enum class DurationLoggerMode { set_reset, toggle };

enum class DurationLoggerChannel : unsigned int {
    channel0,
    channel1,
    channel2,
    channel3,
    channel4,
    channel5,
    channel6,
    channel7,
    channel8,
    channel9,
    channel10,
    channel11,
    channel12,
    channel13,
    channel14,
    channel15,
};

class DurationLogger {
private:
    struct LoggerPin {
        GPIO_T* port;
        uint16_t pin;
    };
    static inline std::array<std::optional<LoggerPin>, 16> _pins;
    const std::optional<LoggerPin> _pin;
    const DurationLoggerMode _mode;
public:
    static OutputPin init_channel(DurationLoggerChannel channel,
                                  Port port,
                                  Pin pin) {
        OutputPin logger_pin({.port = port,
                              .pin = pin,
                              .config = {.pin{},
                                         .mode = GPIO_MODE_OUT,
                                         .speed = GPIO_SPEED_100MHz,
                                         .otype = GPIO_OTYPE_PP,
                                         .pupd = GPIO_PUPD_NOPULL},
                              .altfunc{},
                              .active_state = mcu::gpio::active_state::high});
        _pins[std::to_underlying(channel)] = {
            impl::gpio_instances[std::to_underlying(port)],
            std::to_underlying(pin)};
        return logger_pin;
    }

    DurationLogger(DurationLoggerChannel channel, DurationLoggerMode mode)
            : _pin(_pins[std::to_underlying(channel)]), _mode(mode) {
        if (!_pin.has_value()) {
            return;
        }

        if (_mode == DurationLoggerMode::set_reset) {
            write_reg<uint16_t>(_pin->port->BSCL, _pin->pin);
        } else {
            uint16_t odr_reg =
                    static_cast<uint16_t>(read_reg(_pin->port->ODATA));
            write_reg<uint16_t>(_pin->port->BSCL, ~odr_reg & _pin->pin);
            write_reg<uint16_t>(_pin->port->BSCH, odr_reg & _pin->pin);

            odr_reg = static_cast<uint16_t>(read_reg(_pin->port->ODATA));
            write_reg<uint16_t>(_pin->port->BSCL, ~odr_reg & _pin->pin);
            write_reg<uint16_t>(_pin->port->BSCH, odr_reg & _pin->pin);
        }
    }

    ~DurationLogger() {
        if (!_pin.has_value()) {
            return;
        }

        if (_mode == DurationLoggerMode::set_reset) {
            write_reg<uint16_t>(_pin->port->BSCH, _pin->pin);
        } else {
            uint16_t odr_reg =
                    static_cast<uint16_t>(read_reg(_pin->port->ODATA));
            write_reg<uint16_t>(_pin->port->BSCL, ~odr_reg & _pin->pin);
            write_reg<uint16_t>(_pin->port->BSCH, odr_reg & _pin->pin);
        }
    }
};

} // namespace gpio
} // namespace apm32
} // namespace mcu

#endif
#endif
