#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/uart/uart.h>


namespace mcu {


namespace uart {


Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _rx_pin.init({.port = rx_pin_config.port, 
                  .pin = {.Pin = rx_pin_config.pin,
                          .Mode = GPIO_MODE_AF_PP,
                          .Pull = GPIO_PULLUP,
                          .Speed = GPIO_SPEED_FREQ_HIGH,
                          .Alternate = rx_pin_config.af_selection},
                  .active_state = emb::gpio::ActiveState::high});
            
    _tx_pin.init({.port = tx_pin_config.port,
                  .pin = {.Pin = tx_pin_config.pin,
                          .Mode = GPIO_MODE_AF_PP,
                          .Pull = GPIO_PULLUP,
                          .Speed = GPIO_SPEED_FREQ_HIGH,
                          .Alternate = tx_pin_config.af_selection},
                  .active_state = emb::gpio::ActiveState::high});

    _enable_clk(_peripheral);
    _reg = impl::uart_instances[std::to_underlying(_peripheral)];

    if (config.hal_config.WordLength == UART_WORDLENGTH_9B) {
        if (config.hal_config.Parity == UART_PARITY_NONE) {
            _datamask = 0x1FF;
        } else {
            _datamask = 0xFF;
        }
    } else if (config.hal_config.WordLength == UART_WORDLENGTH_8B) {
        if (config.hal_config.Parity == UART_PARITY_NONE) {
            _datamask = 0xFF;
        } else {
            _datamask = 0x7FF;
        }
    } else if (config.hal_config.WordLength == UART_WORDLENGTH_7B) {
        if (config.hal_config.Parity == UART_PARITY_NONE) {
            _datamask = 0x7F;
        } else {
            _datamask = 0x3F;
        }
    } else {
        _datamask = 0x00;
    }

    _handle.Instance = _reg;
    _handle.Init = config.hal_config;
    _handle.AdvancedInit = config.hal_adv_config;
    if (HAL_UART_Init(&_handle) != HAL_OK) {
        fatal_error("UART module initialization failed");
    }
}	


void Module::_enable_clk(Peripheral peripheral) {
    auto uart_idx = std::to_underlying(peripheral);
    if (!_clk_enabled[uart_idx]) {
        impl::uart_clk_enable_funcs[uart_idx]();
        _clk_enabled[uart_idx] = true;
    }
}


} // namespace uart


} // namespace mcu


#endif
#endif
