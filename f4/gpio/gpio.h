#pragma once


#ifdef STM32F4xx

#include "../mcudef.h"
#include <emblib/core.h>
#include <emblib/interfaces/gpio.h>
#include <algorithm>
#include <array>
#include <utility>


extern "C" void EXTI0_IRQHandler();
extern "C" void EXTI1_IRQHandler();
extern "C" void EXTI2_IRQHandler();
extern "C" void EXTI3_IRQHandler();
extern "C" void EXTI4_IRQHandler();


namespace mcu {

namespace gpio {

struct Config {
    GPIO_TypeDef* port;
    GPIO_InitTypeDef pin;
    emb::gpio::ActiveState active_state;
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
        auto port_idx = std::distance(gpio_ports.begin(), std::find(gpio_ports.begin(), gpio_ports.end(), config.port));
        if (!_clk_enabled[port_idx]) {
            gpio_clk_enable_funcs[port_idx]();
            _clk_enabled[port_idx] = true;
        }	

        _config = config;
        HAL_GPIO_Init(_config.port, &_config.pin);
        _initialized = true;
    }

    void deinit() {	
        if (_initialized) {
            HAL_GPIO_DeInit(_config.port, _config.pin.Pin);
            _initialized = false;
        }
    }

    const Config& config() const { return _config; }
    unsigned int pin_no() const { return POSITION_VAL(_config.pin.Pin); }
    uint16_t pin_bit() const { return static_cast<uint16_t>(_config.pin.Pin); }
    const GPIO_TypeDef* port() const { return _config.port; }
};

} // namespace impl


class Input : public emb::gpio::InputInterface, public impl::Gpio {
    friend void ::EXTI0_IRQHandler();
    friend void ::EXTI1_IRQHandler();
    friend void ::EXTI2_IRQHandler();
    friend void ::EXTI3_IRQHandler();
    friend void ::EXTI4_IRQHandler();
    friend void ::HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
public:
    Input() = default;
    Input(const Config& config) {
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert_param(_initialized);
        if ((mcu::read_reg(_config.port->IDR) & _config.pin.Pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual emb::gpio::State read() const override {
        assert_param(_initialized);
        auto level = read_level();
        return static_cast<emb::gpio::State>(1 - (level ^ std::to_underlying(_config.active_state)));
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
    void init_interrupt(void(*handler)(void), IrqPriority priority) {
        switch (_config.pin.Pin) {
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


class Output : public emb::gpio::OutputInterface, public impl::Gpio {
public:
    Output() = default;
    Output(const Config& config) {
        init(config);
    }

    virtual unsigned int read_level() const override {
        assert_param(_initialized);
        if ((mcu::read_reg(_config.port->IDR) & _config.pin.Pin) != 0) {
            return 1;
        }
        return 0;
    }

    virtual void set_level(unsigned int level) override {
        assert_param(_initialized);
        if(level != 0) {
            mcu::write_reg(_config.port->BSRR, _config.pin.Pin);
        } else {
            mcu::write_reg(_config.port->BSRR, _config.pin.Pin << 16);
        }
    }

    virtual emb::gpio::State read() const override {
        assert_param(_initialized);
        auto level = read_level();
        return static_cast<emb::gpio::State>(1 - (level ^ std::to_underlying(_config.active_state)));
    }

    virtual void set(emb::gpio::State state = emb::gpio::State::active) override {
        assert_param(_initialized);
        auto level = 1 - (std::to_underlying(state) ^ std::to_underlying(_config.active_state));
        set_level(level);
    }

    virtual void reset() override {
        assert_param(_initialized);
        set(emb::gpio::State::inactive);
    }

    virtual void toggle() override {
        assert_param(_initialized);
        auto odr_reg = mcu::read_reg(_config.port->ODR);
        mcu::write_reg(_config.port->BSRR, ((odr_reg & _config.pin.Pin) << 16) | (~odr_reg & _config.pin.Pin));
    }
};


enum class DurationLoggerMode {
    set_reset,
    toggle
};


template <DurationLoggerMode Mode = DurationLoggerMode::set_reset>
class DurationLogger {
private:
    GPIO_TypeDef* _port;
    uint16_t _pin;
public:
    DurationLogger(Output& gpio_output)
            : _port(gpio_output.port())
            , _pin(gpio_output.pin_bit()) {
        if constexpr (Mode == DurationLoggerMode::set_reset) {
            _port->BSRR = _pin;	//HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_SET);
        } else {
            _port->BSRR = ((_port->ODR & _pin) << 16) | (~_port->ODR & _pin);	//HAL_GPIO_TogglePin(m_port, m_pin);
            _port->BSRR = ((_port->ODR & _pin) << 16) | (~_port->ODR & _pin);	//HAL_GPIO_TogglePin(m_port, m_pin);
        }
    }

    DurationLogger(GPIO_TypeDef* port, uint16_t pin_bit)
            : _port(port)
            , _pin(pin_bit) {
        if constexpr (Mode == DurationLoggerMode::set_reset)
        {
            _port->BSRR = _pin;	//HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_SET);
        } else {
            _port->BSRR = ((_port->ODR & _pin) << 16) | (~_port->ODR & _pin);	//HAL_GPIO_TogglePin(m_port, m_pin);
            _port->BSRR = ((_port->ODR & _pin) << 16) | (~_port->ODR & _pin);	//HAL_GPIO_TogglePin(m_port, m_pin);
        }
    }

    ~DurationLogger() {
        if constexpr (Mode == DurationLoggerMode::set_reset) {
            _port->BSRR = static_cast<uint32_t>(_pin) << 16;	//HAL_GPIO_WritePin(m_port, m_pin, GPIO_PIN_RESET);
        } else {
            _port->BSRR = ((_port->ODR & _pin) << 16) | (~_port->ODR & _pin);	//HAL_GPIO_TogglePin(m_port, m_pin);
        }
    }

    static Output init(GPIO_TypeDef* port, uint32_t pin) {
        return Output({	
            .port = port,
            .pin = {
                .Pin = pin,
                .Mode = GPIO_MODE_OUTPUT_PP,
                .Pull = GPIO_NOPULL,
                .Speed = GPIO_SPEED_FREQ_HIGH,
                .Alternate = 0
            },
            .active_state = emb::gpio::ActiveState::high
        });
    }
};

} // namespace gpio

} // namespace mcu

#endif
