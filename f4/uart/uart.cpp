#ifdef STM32F4xx

#include <mculib_stm32/f4/uart/uart.h>


namespace mcu {

namespace uart {

Module::Module(Peripheral peripheral, const RxPinConfig& rx_pin_config, const TxPinConfig& tx_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _rx_pin.init({
        .port = rx_pin_config.port, 
        .pin = {
            .Pin = rx_pin_config.pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = rx_pin_config.af_selection
        },
        .active_state = emb::gpio::ActiveState::high});
            
    _tx_pin.init({
        .port = tx_pin_config.port,
        .pin = {
            .Pin = tx_pin_config.pin,
            .Mode = GPIO_MODE_AF_PP,
            .Pull = GPIO_PULLUP,
            .Speed = GPIO_SPEED_FREQ_HIGH,
            .Alternate = tx_pin_config.af_selection
        },
        .active_state = emb::gpio::ActiveState::high});

    enable_clk();
    _handle.Instance = impl::uart_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_init;
    if (HAL_UART_Init(&_handle) != HAL_OK) {
        fatal_error("UART module initialization failed");
    }
}	

} // namespace uart

} // namespace mcu

#endif
