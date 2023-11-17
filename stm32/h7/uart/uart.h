#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>
#include <emblib/interfaces/tty.h>
#include <utility>


namespace mcu {


namespace uart {


constexpr size_t peripheral_count = 8;
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
    UART_AdvFeatureInitTypeDef hal_adv_config;
};


namespace impl {


inline const std::array<USART_TypeDef*, peripheral_count> uart_instances = {USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8};


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


} // namespace impl


class Module : public emb::Tty, public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    UART_HandleTypeDef _handle = {};
    USART_TypeDef* _reg;

    mcu::gpio::Input _rx_pin;
    mcu::gpio::Output _tx_pin;

    uint32_t _datamask{0};

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
        if (bit_is_clear<uint32_t>(_reg->ISR, USART_ISR_RXNE_RXFNE)) {
            return EOF;
        }
        return read_reg(_reg->RDR) & _datamask;
    }

    virtual int putchar(int ch) override {
        if (bit_is_clear<uint32_t>(_reg->ISR, USART_ISR_TXE_TXFNF)) {
            return EOF;
        }
        write_reg<uint32_t>(_reg->TDR, ch);
        return ch;
    }

private:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace uart


} // namespace mcu


#endif
#endif
