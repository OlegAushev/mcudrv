#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_usart.h>

#include <mcudrv/apm32/f4/apm32f4_base.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/generic/uart.hpp>

namespace mcu {
namespace apm32 {
namespace usart {

enum class Peripheral : unsigned int {
    usart1,
    usart2,
    usart3,
    uart4,
    uart5,
    usart6,
};

constexpr size_t peripheral_count = 6;

struct RxPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};

struct TxPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};

struct Config {
    USART_Config_T hal_config;
};

namespace impl {

inline const std::array<USART_T*, peripheral_count> usart_instances = {
    USART1, USART2, USART3, UART4, UART5, USART6};

inline Peripheral to_peripheral(const USART_T* instance) {
    return static_cast<Peripheral>(std::distance(
            usart_instances.begin(),
            std::find(
                    usart_instances.begin(), usart_instances.end(), instance)));
}

inline std::array<void (*)(void), peripheral_count> usart_clk_enable_funcs = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USART2); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_USART3); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_UART4); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_UART5); },
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_USART6); },
};

} // namespace impl

class Module : public mcu::uart::tty,
               public emb::singleton_array<Module, peripheral_count>,
               private emb::noncopyable {
private:
    const Peripheral _peripheral;
    USART_T* _reg;
    mcu::gpio::AlternatePin _rx_pin;
    mcu::gpio::AlternatePin _tx_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral,
           const RxPinConfig& rx_pin_config,
           const TxPinConfig& tx_pin_config,
           Config config);
    Peripheral peripheral() const { return _peripheral; }
    static Module* instance(Peripheral peripheral) {
        return emb::singleton_array<Module, peripheral_count>::instance(
                std::to_underlying(peripheral));
    }

    virtual int getchar() override {
        if (bit_is_clear<uint32_t>(_reg->STS, USART_FLAG_RXBNE)) {
            return EOF;
        }
        return _reg->DATA_B.DATA;
    }

    virtual int putchar(int ch) override {
        if (bit_is_clear<uint32_t>(_reg->STS, USART_FLAG_TXBE)) {
            return EOF;
        }
        _reg->DATA_B.DATA = static_cast<uint16_t>(ch) & 0x1FF;
        return ch;
    }

    // TODO
    // virtual int recv(char* buf, size_t len) override {
    //     int i = 0;
    //     char ch = 0;

    //     while ((i < len) && (getchar(ch) == 1)) {
    //         buf[i++] = ch;
    //     }
    //     return i;
    // }

    // TODO
    // virtual int send(const char* buf, size_t len) override {
    //     if (HAL_UART_Transmit(&_handle, reinterpret_cast<const uint8_t*>(buf), static_cast<uint16_t>(len), timeout_ms) != HAL_OK) {
    //         return 0;
    //     }
    //     return 1;
    // }
protected:
    static void _enable_clk(Peripheral peripheral);
};

} // namespace usart
} // namespace apm32
} // namespace mcu

#endif
#endif
