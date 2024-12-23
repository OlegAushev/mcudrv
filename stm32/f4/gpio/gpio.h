#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/stm32_f4_base.h>
#include <mcudrv/stm32/f4/system/system.h>
#include <emblib/core.hpp>
#include <emblib/interfaces/gpio.h>
#include <algorithm>
#include <array>
#include <optional>
#include <utility>


extern "C" void EXTI0_IRQHandler();
extern "C" void EXTI1_IRQHandler();
extern "C" void EXTI2_IRQHandler();
extern "C" void EXTI3_IRQHandler();
extern "C" void EXTI4_IRQHandler();


namespace mcu {
namespace gpio {


struct PinConfig {
    GPIO_TypeDef* port;
    GPIO_InitTypeDef pin;
    emb::gpio::active_pin_state actstate;
};


namespace impl {


constexpr size_t port_count = 8;


inline const std::array<GPIO_TypeDef*, port_count> gpio_ports = {
    GPIOA, GPIOB, GPIOC, GPIOD,
    GPIOE, GPIOF, GPIOG, GPIOH
};


inline std::array<void(*)(void), port_count> gpio_clk_enable_funcs = {
    [](){ __HAL_RCC_GPIOA_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOB_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOC_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOD_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOE_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOF_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOG_CLK_ENABLE(); },
    [](){ __HAL_RCC_GPIOH_CLK_ENABLE(); }
};


class GpioPin
{
private:
    static inline std::array<uint16_t, port_count> _assigned{};
    static inline std::array<bool, port_count> _clk_enabled{};
protected:
    bool _initialized{false};
    GPIO_TypeDef* _port;
    uint16_t _pin;
    std::optional<emb::gpio::active_pin_state> _actstate{std::nullopt};
    GpioPin() = default;
public:
    void init(PinConfig config) {
        size_t port_idx = static_cast<size_t>(std::distance(gpio_ports.begin(),
                                                            std::find(gpio_ports.begin(), gpio_ports.end(), config.port)));
        if (_assigned[port_idx] & config.pin.Pin) {
            fatal_error();
        }
        _assigned[port_idx] |= static_cast<uint16_t>(config.pin.Pin);

        if (!_clk_enabled[port_idx]) {
            gpio_clk_enable_funcs[port_idx]();
            _clk_enabled[port_idx] = true;
        }

        _port = config.port;
        _pin = static_cast<uint16_t>(config.pin.Pin);
        _actstate = config.actstate;

        HAL_GPIO_Init(config.port, &config.pin);
        _initialized = true;
    }

    void deinit() {
        if (_initialized) {
            HAL_GPIO_DeInit(_port, _pin);
            _initialized = false;
        }
    }

    unsigned int pin_no() const { return POSITION_VAL(_pin); }
    uint16_t pin_bit() const { return _pin; }
    const GPIO_TypeDef* port() const { return _port; }
};


} // namespace impl


class InputPin : public mcu::gpio::input_pin, public impl::GpioPin {
    friend void ::EXTI0_IRQHandler();
    friend void ::EXTI1_IRQHandler();
    friend void ::EXTI2_IRQHandler();
    friend void ::EXTI3_IRQHandler();
    friend void ::EXTI4_IRQHandler();
    friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
public:
    InputPin() = default;
    InputPin(const PinConfig& config) {
        assert(config.pin.Mode == GPIO_MODE_INPUT
            || config.pin.Mode == GPIO_MODE_IT_RISING
            || config.pin.Mode == GPIO_MODE_IT_FALLING
            || config.pin.Mode == GPIO_MODE_IT_RISING_FALLING
            || config.pin.Mode == GPIO_MODE_EVT_RISING
            || config.pin.Mode == GPIO_MODE_EVT_FALLING
            || config.pin.Mode == GPIO_MODE_EVT_RISING_FALLING);
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert(_initialized);
        if ((mcu::read_reg(_port->IDR) & _pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual mcu::gpio::pin_state read() const override {
        assert(_initialized);
        if (read_level() == std::to_underlying(*_actstate)) {
            return mcu::gpio::pin_state::active;
        }
        return mcu::gpio::pin_state::inactive;
    }
private:
    IRQn_Type _irqn = NonMaskableInt_IRQn;	// use NonMaskableInt_IRQn as value for not initialized interrupt
    static inline std::array<void(*)(void), 16> on_interrupt = {
        emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
        emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
        emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
        emb::invalid_function, emb::invalid_function, emb::invalid_function, emb::invalid_function,
    };
public:
    void init_interrupts(void(*handler)(void), IrqPriority priority) {
        switch (_pin) {
        case GPIO_PIN_0:
            _irqn = EXTI0_IRQn;
            break;
        case GPIO_PIN_1:
            _irqn = EXTI1_IRQn;
            break;
        case GPIO_PIN_2:
            _irqn = EXTI2_IRQn;
            break;
        case GPIO_PIN_3:
            _irqn = EXTI3_IRQn;
            break;
        case GPIO_PIN_4:
            _irqn = EXTI4_IRQn;
            break;
        case GPIO_PIN_5: case GPIO_PIN_6: case GPIO_PIN_7: case GPIO_PIN_8: case GPIO_PIN_9:
            _irqn = EXTI9_5_IRQn;
            break;
        case GPIO_PIN_10: case GPIO_PIN_11: case GPIO_PIN_12: case GPIO_PIN_13: case GPIO_PIN_14: case GPIO_PIN_15:
            _irqn = EXTI15_10_IRQn;
            break;
        default:
            _irqn = NonMaskableInt_IRQn;
            return;
        }
        HAL_NVIC_SetPriority(_irqn, priority.get(), 0);
        on_interrupt[this->pin_no()] = handler;
    }

    void enable_interrupts() {
        if (_irqn != NonMaskableInt_IRQn) {
            HAL_NVIC_EnableIRQ(_irqn);
        }
    }

    void disable_interrupts() {
        if (_irqn != NonMaskableInt_IRQn) {
            HAL_NVIC_EnableIRQ(_irqn);
        }
    }
};


class OutputPin : public mcu::gpio::output_pin, public impl::GpioPin {
public:
    OutputPin() = default;
    OutputPin(const PinConfig& config) {
        assert(config.pin.Mode == GPIO_MODE_OUTPUT_PP
            || config.pin.Mode == GPIO_MODE_OUTPUT_OD);
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert(_initialized);
        if ((mcu::read_reg(_port->IDR) & _pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual void set_level(unsigned int level) override {
        assert(_initialized);
        if(level != 0) {
            mcu::write_reg(_port->BSRR, static_cast<uint32_t>(_pin));
        } else {
            mcu::write_reg(_port->BSRR, static_cast<uint32_t>(_pin) << 16u);
        }
    }

    virtual mcu::gpio::pin_state read() const override {
        assert(_initialized);
        if (read_level() == std::to_underlying(*_actstate)) {
            return mcu::gpio::pin_state::active;
        }
        return mcu::gpio::pin_state::inactive;
    }

    virtual void set(mcu::gpio::pin_state s = mcu::gpio::pin_state::active) override {
        assert(_initialized);
        if (s == mcu::gpio::pin_state::active) {
            set_level(std::to_underlying(*_actstate));
        } else {
            set_level(1u - std::to_underlying(*_actstate));
        }
    }

    virtual void reset() override {
        assert(_initialized);
        set(mcu::gpio::pin_state::inactive);
    }

    virtual void toggle() override {
        assert(_initialized);
        uint32_t odr_reg = mcu::read_reg(_port->ODR);
        mcu::write_reg(_port->BSRR, ((odr_reg & _pin) << 16u) | (~odr_reg & _pin));
    }
};


class AlternatePin : public impl::GpioPin {
public:
    AlternatePin() = default;
    AlternatePin(const PinConfig& config) {
        assert(config.pin.Mode == GPIO_MODE_AF_PP || config.pin.Mode == GPIO_MODE_AF_OD);
        init(config);
    }
};


class AnalogPin : public impl::GpioPin {
public:
    AnalogPin() = default;
    AnalogPin(const PinConfig& config) {
        assert(config.pin.Mode == GPIO_MODE_ANALOG);
        init(config);
    }
};




enum class DurationLoggerMode {
    set_reset,
    toggle
};


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
        GPIO_TypeDef* port;
        uint16_t pin;
    };
    static inline std::array<std::optional<LoggerPin>, 16> _pins;
    const std::optional<LoggerPin> _pin;
    const DurationLoggerMode _mode;
public:
    static OutputPin init_channel(DurationLoggerChannel channel, GPIO_TypeDef* port, uint16_t pin) {
        OutputPin logger_pin({.port = port,
                              .pin = {
                                    .Pin = pin,
                                    .Mode = GPIO_MODE_OUTPUT_PP,
                                    .Pull = GPIO_NOPULL,
                                    .Speed = GPIO_SPEED_FREQ_HIGH,
                                    .Alternate = 0},
                              .actstate = emb::gpio::active_pin_state::high});
        _pins[std::to_underlying(channel)] = {port, pin};
        return logger_pin;
    }

    DurationLogger(DurationLoggerChannel channel, DurationLoggerMode mode)
            : _pin(_pins[std::to_underlying(channel)])
            , _mode(mode) {
        if (!_pin.has_value()) {
            return;
        }

        if (_mode == DurationLoggerMode::set_reset) {
            _pin->port->BSRR = _pin->pin;
        } else {
            uint32_t odr_reg = mcu::read_reg(_pin->port->ODR);
            mcu::write_reg(_pin->port->BSRR, ((odr_reg & _pin->pin) << 16u) | (~odr_reg & _pin->pin));
            odr_reg = mcu::read_reg(_pin->port->ODR);
            mcu::write_reg(_pin->port->BSRR, ((odr_reg & _pin->pin) << 16u) | (~odr_reg & _pin->pin));
        }
    }

    ~DurationLogger() {
        if (!_pin.has_value()) {
            return;
        }

        if (_mode == DurationLoggerMode::set_reset) {
            _pin->port->BSRR = static_cast<uint32_t>(_pin->pin) << 16;
        } else {
            uint32_t odr_reg = mcu::read_reg(_pin->port->ODR);
            mcu::write_reg(_pin->port->BSRR, ((odr_reg & _pin->pin) << 16u) | (~odr_reg & _pin->pin));
        }
    }
};


} // namespace gpio
} // namespace mcu


#endif
#endif
