#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include "adc.h"


namespace mcu {


namespace adc {


Module::Module(Peripheral peripheral, Config config, dma::Stream* dma)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk(peripheral);
    _reg = impl::adc_instances[std::to_underlying(_peripheral)];

    ADC_CommonConfig(&config.hal_common_config);
    ADC_Config(_reg, &config.hal_config);

    if (config.eoc_on_each_conv) {
        _reg->CTRL2_B.EOCSEL = 1;
    }

    _reg->CTRL2_B.ADCEN = 1;
    auto counter = (3 * (core_clk_freq() / 1000000));
    while(counter != 0)
    {
        --counter;
    }

    if (dma) {
        _reg->CTRL2_B.DMAEN = 1;
        if (config.dma_continuous_requests) {
            _reg->CTRL2_B.DMADISSEL = 1;
        }

        write_reg(dma->stream_reg()->PADDR, uint32_t(&(_reg->REGDATA)));
    }
}


void Module::init_injected_channel(const PinConfig& pin_config, const InjectedChannelConfig& channel_config) {
    mcu::gpio::Config cfg{.port = pin_config.port,
                          .pin = {.pin = pin_config.pin,
                                  .mode = GPIO_MODE_AN,
                                  .speed{},
                                  .otype{},
                                  .pupd = GPIO_PUPD_NOPULL},
                           .af_selection{},
                           .active_state{}};
    mcu::gpio::AnalogIO input(cfg);

    // ADC_ConfigInjectedChannel(_reg, channel_config.channel, channel_config.rank, channel_config.sampletime);
    // ADC_ConfigInjectedOffset(_reg, channel_config.channel, channel_config.offset);
}


void Module::init_regular_channel(const PinConfig& pin_config, const RegularChannelConfig& channel_config) {
    mcu::gpio::Config cfg{.port = pin_config.port,
                          .pin = {.pin = pin_config.pin,
                                  .mode = GPIO_MODE_AN,
                                  .speed{},
                                  .otype{},
                                  .pupd = GPIO_PUPD_NOPULL},
                           .af_selection{},
                           .active_state{}};
    mcu::gpio::AnalogIO input(cfg);

    for (auto rank : channel_config.ranks) {
        ADC_ConfigRegularChannel(_reg, channel_config.channel, rank, channel_config.sampletime);
    }
}


void Module::init_interrupts(uint32_t interrupt_list, mcu::IrqPriority priority) {
    _reg->STS_B.AWDFLG = 0;
    _reg->STS_B.EOCFLG = 0;
    _reg->STS_B.INJEOCFLG = 0;
    _reg->STS_B.INJCSFLG = 0;
    _reg->STS_B.REGCSFLG = 0;
    _reg->STS_B.OVREFLG = 0;

    set_bit(_reg->CTRL1, interrupt_list);
    mcu::set_irq_priority(ADC_IRQn, priority);
}


void Module::_enable_clk(Peripheral peripheral) {
    auto adc_idx = std::to_underlying(peripheral);
    if (_clk_enabled[adc_idx]) {
        return;
    }

    impl::adc_clk_enable_funcs[adc_idx]();
    _clk_enabled[adc_idx] = true;
}


}


} // namespace mcu


#endif
#endif
