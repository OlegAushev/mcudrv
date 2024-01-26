#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/i2c/i2c.h>


namespace mcu {


namespace i2c {


Module::Module(Peripheral peripheral, const SdaPinConfig& sda_pin_config, const SclPinConfig& scl_pin_config, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _sda_pin.init({.port = sda_pin_config.port, 
                  .pin = {.Pin = sda_pin_config.pin,
                          .Mode = GPIO_MODE_AF_OD,
                          .Pull = GPIO_NOPULL, // external pullup is required
                          .Speed = GPIO_SPEED_FREQ_HIGH,
                          .Alternate = sda_pin_config.af_selection},
                  .actstate = emb::gpio::active_state::high});
            
    _scl_pin.init({.port = scl_pin_config.port,
                  .pin = {.Pin = scl_pin_config.pin,
                          .Mode = GPIO_MODE_AF_OD,
                          .Pull = GPIO_NOPULL, // external pullup is required
                          .Speed = GPIO_SPEED_FREQ_HIGH,
                          .Alternate = scl_pin_config.af_selection},
                  .actstate = emb::gpio::active_state::high});

    _enable_clk(_peripheral);
    _reg = impl::i2c_instances[std::to_underlying(_peripheral)];
    _handle.Instance = _reg;
    _handle.Init = config.hal_config;
    if (HAL_I2C_Init(&_handle) != HAL_OK) {
        fatal_error("I2C module initialization failed");
    }
}


void Module::_enable_clk(Peripheral peripheral) {
    auto i2c_idx = std::to_underlying(peripheral);
    if (!_clk_enabled[i2c_idx]) {
        impl::uart_clk_enable_funcs[i2c_idx]();
        _clk_enabled[i2c_idx] = true;
    }
}


} // namespace i2c


} // namespace mcu


#endif
#endif
