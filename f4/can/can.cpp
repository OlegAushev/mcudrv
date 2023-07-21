#ifdef STM32F4xx

#include <mculib_stm32/f4/can/can.h>


namespace mcu {

namespace can {

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

    _handle.Instance = impl::can_instances[static_cast<size_t>(_peripheral)];
    _handle.Init = config.hal_init;

    if(HAL_CAN_Init(&_handle) != HAL_OK) {
        fatal_error("CAN module initialization failed");
    }
}


void Module::enable_clk() {
    auto can_idx = std::to_underlying(_peripheral);
    if (!_clk_enabled[can_idx]) {
        impl::can_clk_enable_funcs[can_idx]();
        _clk_enabled[can_idx] = true;
    }
}

} // namespace can

} // namespace mcu

#endif
