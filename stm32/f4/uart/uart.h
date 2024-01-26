#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include "../stm32_f4_base.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/interfaces/tty.h>

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
};


constexpr size_t peripheral_count = 6;


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
    UART_InitTypeDef hal_config;
};


namespace impl {

inline const std::array<USART_TypeDef*, peripheral_count> uart_instances = {USART1, USART2, USART3, UART4, UART5, USART6};


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
};

}


class Module : public emb::tty, public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    UART_HandleTypeDef _handle = {};
    USART_TypeDef* _reg;

    mcu::gpio::AlternateIO _rx_pin;
    mcu::gpio::AlternateIO _tx_pin;

    uint32_t _rdatamask{0};
    uint32_t _wdatamask{0};

    static inline std::array<bool, peripheral_count> _clk_enabled = {};
public:
    Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config);
    
    Peripheral peripheral() const { return _peripheral; }
    UART_HandleTypeDef* handle() { return &_handle; }
    USART_TypeDef* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    virtual int getchar() override {
        //HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout)
        if (bit_is_clear<uint32_t>(_reg->SR, USART_SR_RXNE)) {
            return EOF;
        }
        return static_cast<int>(read_reg(_reg->DR) & _rdatamask);
    }

    virtual int putchar(int ch) override {
        if (bit_is_clear<uint32_t>(_reg->SR, USART_SR_TXE)) {
            return EOF;
        }
        write_reg<uint32_t>(_reg->DR, static_cast<uint32_t>(ch) & _wdatamask);
        return ch;
    }

private:
    static void _enable_clk(Peripheral peripheral);
};



} // namespace uart


} // namespace mcu


#endif
#endif
