#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/dac/dac.h>


namespace mcu {
namespace dac {


Module::Module(Peripheral peripheral)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk(peripheral);
    _reg = impl::dac_instances[std::to_underlying(peripheral)];
    _handle.Instance = _reg;

    if (HAL_DAC_Init(&_handle) != HAL_OK) {
        fatal_error("ADC module initialization failed");
    }
}


void Module::initialize_channel(Channel channel, const PinConfig& pin_config, ChannelConfig config) {
    mcu::gpio::Config cfg = {};
    cfg.port = pin_config.port;
    cfg.pin.Pin = pin_config.pin;
    cfg.pin.Mode = GPIO_MODE_ANALOG;
    cfg.pin.Pull = GPIO_NOPULL;
    mcu::gpio::AnalogPin output(cfg);
    
    if (HAL_DAC_ConfigChannel(&_handle, &config.hal_config, std::to_underlying(channel)) != HAL_OK) {
        fatal_error("DAC channel initialization failed");
    }

    HAL_DAC_Start(&_handle, std::to_underlying(channel));
}


void Module::_enable_clk(Peripheral peripheral) {
    auto dac_idx = std::to_underlying(peripheral);
    if (_clk_enabled[dac_idx]) {
        return;
    }

    impl::dac_clk_enable_funcs[dac_idx]();
    _clk_enabled[dac_idx] = true;
}


} // namespace dac
} // namespace mcu


#endif
#endif
