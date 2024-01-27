#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "dac.h"


namespace mcu {

    
namespace dac {


Module::Module(Peripheral peripheral)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _enable_clk(peripheral);
    _reg = impl::dac_instances[std::to_underlying(peripheral)];
}


void Module::init_channel(Channel channel, const PinConfig& pin_config, ChannelConfig config) {
    mcu::gpio::Config cfg{.port = pin_config.port,
                          .pin = {.pin = pin_config.pin,
                                  .mode = GPIO_MODE_AN,
                                  .speed{},
                                  .otype{},
                                  .pupd = GPIO_PUPD_NOPULL},
                           .af_selection{},
                           .actstate{}};
    mcu::gpio::AnalogPin output(cfg);

    DAC_Config(std::to_underlying(channel), &config.hal_config);
    DAC_Enable(static_cast<DAC_CHANNEL_T>(channel));
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
