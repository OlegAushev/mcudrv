#pragma once


#include <gd32f4xx_gpio.h>
#ifdef MCUDRV_GD32
#ifdef GD32F4xx


#include "../gd32_f4_base.h"
#include <emblib/interfaces/gpio.h>
#include <algorithm>
#include <array>
#include <utility>


namespace mcu {


namespace gpio {


struct Config {
    uint32_t port;
    uint32_t pin;
    uint32_t mode;
    uint32_t pull;
    // GPIO_AF_T af_selection;
    emb::gpio::active_state active_state;
};


namespace impl {


constexpr size_t port_count = 9;


inline const std::array<uint32_t, port_count> gpio_ports = {
    GPIOA, GPIOB, GPIOC, GPIOD,
    GPIOE, GPIOF, GPIOG, GPIOH,
    GPIOI
};


inline std::array<void(*)(void), port_count> gpio_clk_enable_funcs = {
    [](){ rcu_periph_clock_enable(RCU_GPIOA); },
    [](){ rcu_periph_clock_enable(RCU_GPIOB); },
    [](){ rcu_periph_clock_enable(RCU_GPIOC); },
    [](){ rcu_periph_clock_enable(RCU_GPIOD); },
    [](){ rcu_periph_clock_enable(RCU_GPIOE); },
    [](){ rcu_periph_clock_enable(RCU_GPIOF); },
    [](){ rcu_periph_clock_enable(RCU_GPIOG); },
    [](){ rcu_periph_clock_enable(RCU_GPIOH); },
    [](){ rcu_periph_clock_enable(RCU_GPIOI); }
};


class Gpio
{
private:
    static inline std::array<bool, port_count> _clk_enabled{};
protected:
    Config _config;
    bool _initialized{false};
    Gpio() = default;
public:
    void init(const Config& config) {
        // enable port clock
        size_t port_idx = static_cast<size_t>(std::distance(gpio_ports.begin(), 
                                                            std::find(gpio_ports.begin(), gpio_ports.end(), config.port)));
        if (!_clk_enabled[port_idx]) {
            gpio_clk_enable_funcs[port_idx]();
            _clk_enabled[port_idx] = true;
        }
        _config = config;

        // if (_config.pin.mode == GPIO_MODE_AF) {
        //     GPIO_ConfigPinAF(_config.port, static_cast<GPIO_PIN_SOURCE_T>(bit_position(_config.pin.pin)), _config.af_selection);
        // }

        gpio_mode_set(_config.port, _config.mode, _config.pull, _config.pin);

        _initialized = true;
    }

    const Config& config() const { return _config; }
    unsigned int pin_no() const { return __CLZ(__RBIT(_config.pin)); }
    uint16_t pin_bit() const { return static_cast<uint16_t>(_config.pin); }
    uint32_t port() const { return _config.port; }
};


} // namespace impl


class Input : public emb::gpio::Input, public impl::Gpio {
    // friend void ::EXTI0_IRQHandler();
    // friend void ::EXTI1_IRQHandler();
    // friend void ::EXTI2_IRQHandler();
    // friend void ::EXTI3_IRQHandler();
    // friend void ::EXTI4_IRQHandler();
    // friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
public:
    Input() = default;
    Input(const Config& config) {
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert(_initialized);
        if ((read_reg(_config.port->IDATA) & _config.pin.pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual emb::gpio::State read() const override {
        assert(_initialized);
        return (read_level() == std::to_underlying(_config.actstate)) ? emb::gpio::state::active : emb::gpio::state::inactive; 
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
//     void init_interrupt(void(*handler)(void), IrqPriority priority) {
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


class Output : public emb::gpio::Output, public impl::Gpio {
public:
    Output() = default;
    Output(const Config& config) {
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert(_initialized);
        if ((read_reg(_config.port->IDATA) & _config.pin.pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual void set_level(unsigned int level) override {
        assert(_initialized);
        if(level != 0) {
            write_reg(_config.port->BSCL, _config.pin.pin);
        } else {
            write_reg(_config.port->BSCH, _config.pin.pin);
        }
    }

    virtual emb::gpio::State read() const override {
        assert(_initialized);
        return (read_level() == std::to_underlying(_config.actstate)) ? emb::gpio::state::active : emb::gpio::state::inactive;
    }

    virtual void set(emb::gpio::State state = emb::gpio::state::active) override {
        assert(_initialized);
        if (state == emb::gpio::state::active) {
            set_level(std::to_underlying(_config.actstate));
        } else {
            set_level(1 - std::to_underlying(_config.actstate));
        }
    }

    virtual void reset() override {
        assert(_initialized);
        set(emb::gpio::state::inactive);
    }

    virtual void toggle() override {
        assert(_initialized);
        uint16_t odr_reg = static_cast<uint16_t>( read_reg(_config.port->ODATA));
        write_reg<uint16_t>(_config.port->BSCL, ~odr_reg & _config.pin.pin);
        write_reg<uint16_t>(_config.port->BSCH, odr_reg & _config.pin.pin);
    }
};


} // namespace gpio


} // namespace mcu


#endif
#endif
