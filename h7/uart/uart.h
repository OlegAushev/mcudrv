#pragma once


#ifdef STM32H7xx

#include "../mcu_def.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/interfaces/uart.h>

#include <utility>


namespace mcu {

namespace uart {

enum class Peripheral : unsigned int {
    usart1,
    usart2,
    usart3,
    uart4,
    uart5,
    usart6,
    uart7,
    uart8
};


constexpr size_t peripheral_count = 8;


struct RxPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct TxPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct Config {
    UART_InitTypeDef hal_init;
    UART_AdvFeatureInitTypeDef hal_init_advanced;
};


namespace impl {

inline constexpr std::array<USART_TypeDef*, peripheral_count> uart_instances = {USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8};


inline Peripheral to_peripheral(const USART_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(uart_instances.begin(), std::find(uart_instances.begin(), uart_instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> uart_clk_enable_funcs = {
    [](){ __HAL_RCC_USART1_CLK_ENABLE(); },
    [](){ __HAL_RCC_USART2_CLK_ENABLE(); },
    [](){ __HAL_RCC_USART3_CLK_ENABLE(); },
    [](){ __HAL_RCC_UART4_CLK_ENABLE(); },
    [](){ __HAL_RCC_UART5_CLK_ENABLE(); },
    [](){ __HAL_RCC_USART6_CLK_ENABLE(); },
    [](){ __HAL_RCC_UART7_CLK_ENABLE(); },
    [](){ __HAL_RCC_UART8_CLK_ENABLE(); }	
};

}


class Module : public emb::uart::UartInterface, public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    UART_HandleTypeDef _handle = {};
    mcu::gpio::Input _rx_pin;
    mcu::gpio::Output _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled = {};

    static constexpr uint32_t timeout_ms = 1000;
public:
    Module(Peripheral peripheral, const RxPinConfig& rxPinConf, const TxPinConfig& txPinConf, const Config& conf);
    Peripheral peripheral() const { return _peripheral; }
    UART_HandleTypeDef* handle() { return &_handle; }
    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    virtual int getchar(char& ch) override {
        if (HAL_UART_Receive(&_handle, reinterpret_cast<uint8_t*>(&ch), 1, 0) != HAL_OK) {
            return 0;
        }
        return 1;
    }

    virtual int recv(char* buf, size_t len) override {
        int i = 0;
        char ch = 0;

        while ((i < len) && (getchar(ch) == 1)) {
            buf[i++] = ch;
        }
        return i;
    }

    virtual int putchar(char ch) override {
        if (HAL_UART_Transmit(&_handle, reinterpret_cast<const uint8_t*>(&ch), 1, timeout_ms) != HAL_OK) {
            return 0;
        }
        return 1;
    }

    virtual int send(const char* buf, size_t len) override {
        if (HAL_UART_Transmit(&_handle, reinterpret_cast<const uint8_t*>(buf), static_cast<uint16_t>(len), timeout_ms) != HAL_OK) {
            return 0;
        }
        return 1;
    }
protected:
    void enable_clk() {
        auto uart_idx = std::to_underlying(_peripheral);
        if (!_clk_enabled[uart_idx]) {
            impl::uart_clk_enable_funcs[uart_idx]();
            _clk_enabled[uart_idx] = true;
        }
    }
};

} // namespace uart

} // namespace mcu

#endif

