#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv/apm32/f4/adc/adc.hpp>

namespace mcu {
namespace apm32 {
namespace adc {

Module::Module(Peripheral peripheral, Config config, dma::Stream* dma)
        : emb::singleton_array<Module, peripheral_count>(
                  this, std::to_underlying(peripheral)),
          _peripheral(peripheral) {
    _enable_clk(peripheral);
    _reg = impl::adc_instances[std::to_underlying(_peripheral)];

    if (!_common_initialized) {
        ADC_CommonConfig(&config.hal_common_config);
        _common_initialized = true;
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
    _reg->CTRL1_B.RESSEL = config.resolution;
    _reg->CTRL1_B.SCANEN = config.scan_mode;

    _reg->CTRL2_B.REGEXTTRGEN = config.ext_trigger_edge;
    _reg->CTRL2_B.REGEXTTRGSEL = config.ext_trigger;
    _reg->CTRL2_B.DALIGNCFG = config.data_align;
    _reg->CTRL2_B.CONTCEN = config.continuous_mode;

    _reg->REGSEQ1_B.REGSEQLEN = config.conv_num - 1;
#pragma GCC diagnostic pop

    if (config.eoc_on_each_conv) {
        _reg->CTRL2_B.EOCSEL = 1;
    }

    _reg->CTRL2_B.ADCEN = 1;
    auto counter = (3 * (core_clk_freq() / 1000000));
    while (counter != 0) {
        --counter;
    }

    if (dma) {
        _reg->CTRL2_B.DMAEN = 1;
        if (config.dma_continuous_requests) {
            _reg->CTRL2_B.DMADISSEL = 1;
        }

        write_reg(dma->stream_reg()->PADDR, uint32_t(&(_reg->REGDATA)));
    }

    if (config.discontinuous_mode) {
        _reg->CTRL1_B.REGDISCEN = 1;
        if (config.discontinuous_conv_num == 0 ||
            config.discontinuous_conv_num > 8) {
            fatal_error();
        }
        _reg->CTRL1_B.DISCNUMCFG = (config.discontinuous_conv_num - 1) & 0x7;
    }

    // configure injected channels
    if (config.injected.has_value()) {
        if (config.injected.value().conv_num > 4) {
            fatal_error();
        }
        _reg->INJSEQ_B.INJSEQLEN = (config.injected.value().conv_num - 1) & 0x3;
        _reg->CTRL1_B.INJGACEN = config.injected.value().auto_conv;
        _reg->CTRL1_B.INJDISCEN = config.injected.value().discontinuous_mode;
        ADC_ConfigExternalTrigInjectedConvEdge(
                _reg, config.injected.value().ext_trigger_edge);
        ADC_ConfigExternalTrigInjectedConv(_reg,
                                           config.injected.value().ext_trigger);
    }
}

void Module::init_injected(const PinConfig& pin_config,
                           const InjectedChannelConfig& channel_config) {
    gpio::PinConfig cfg{.port = pin_config.port,
                        .pin = pin_config.pin,
                        .config = {.pin{},
                                   .mode = GPIO_MODE_AN,
                                   .speed{},
                                   .otype{},
                                   .pupd = GPIO_PUPD_NOPULL},
                        .altfunc{},
                        .active_state{}};
    gpio::AnalogPin input(cfg);

    for (auto rank : channel_config.ranks) {
        ADC_ConfigInjectedChannel(_reg,
                                  channel_config.channel,
                                  static_cast<ADC_INJEC_CHANNEL_T>(rank),
                                  channel_config.sampletime);
        ADC_ConfigInjectedOffset(_reg,
                                 static_cast<ADC_INJEC_CHANNEL_T>(rank),
                                 channel_config.offset);
    }
}

void Module::init_regular(const PinConfig& pin_config,
                          const RegularChannelConfig& channel_config) {
    gpio::PinConfig cfg{.port = pin_config.port,
                        .pin = pin_config.pin,
                        .config = {.pin{},
                                   .mode = GPIO_MODE_AN,
                                   .speed{},
                                   .otype{},
                                   .pupd = GPIO_PUPD_NOPULL},
                        .altfunc{},
                        .active_state{}};
    gpio::AnalogPin input(cfg);

    for (auto rank : channel_config.ranks) {
        ADC_ConfigRegularChannel(
                _reg, channel_config.channel, rank, channel_config.sampletime);
    }
}

void Module::init_internal_injected(
        const InjectedChannelConfig& channel_config) {
    for (auto rank : channel_config.ranks) {
        ADC_ConfigInjectedChannel(_reg,
                                  channel_config.channel,
                                  static_cast<ADC_INJEC_CHANNEL_T>(rank),
                                  channel_config.sampletime);
        ADC_ConfigInjectedOffset(_reg,
                                 static_cast<ADC_INJEC_CHANNEL_T>(rank),
                                 channel_config.offset);
    }
    ADC_EnableTempSensorVrefint();
}

void Module::init_internal_regular(const RegularChannelConfig& channel_config) {
    for (auto rank : channel_config.ranks) {
        ADC_ConfigRegularChannel(
                _reg, channel_config.channel, rank, channel_config.sampletime);
    }
    ADC_EnableTempSensorVrefint();
}

void Module::init_interrupts(uint32_t interrupt_bitset) {
    _reg->STS_B.AWDFLG = 0;
    _reg->STS_B.EOCFLG = 0;
    _reg->STS_B.INJEOCFLG = 0;
    _reg->STS_B.INJCSFLG = 0;
    _reg->STS_B.REGCSFLG = 0;
    _reg->STS_B.OVREFLG = 0;

    set_bit(_reg->CTRL1, interrupt_bitset);
}

void Module::_enable_clk(Peripheral peripheral) {
    auto adc_idx = std::to_underlying(peripheral);
    if (_clk_enabled[adc_idx]) {
        return;
    }

    impl::adc_clk_enable_funcs[adc_idx]();
    _clk_enabled[adc_idx] = true;
}

} // namespace adc
} // namespace apm32
} // namespace mcu

#endif
#endif
