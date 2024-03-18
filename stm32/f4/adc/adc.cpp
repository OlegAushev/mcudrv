#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/adc/adc.h>


namespace mcu {


namespace adc {


Module::Module(Peripheral peripheral, const Config& config, dma::Stream* dma)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk(peripheral);
    _reg = impl::adc_instances[std::to_underlying(_peripheral)];
    _handle.Instance = _reg;
    _handle.Init = config.hal_config;

    if (HAL_ADC_Init(&_handle) != HAL_OK) {
        fatal_error("ADC module initialization failed");
    }

    set_bit<uint32_t>(_reg->CR2, ADC_CR2_ADON);
    auto counter = (ADC_STAB_DELAY_US * (core_clk_freq() / 1000000));
    while(counter != 0)
    {
        --counter;
    }

    if (dma) {
        set_bit<uint32_t>(_reg->CR2, ADC_CR2_DMA);
        write_reg(dma->stream_reg()->PAR, uint32_t(&(_reg->DR)));
    }
}


void Module::init_injected_channel(const PinConfig& pin_config, InjectedChannelConfig channel_config) {
    mcu::gpio::Config cfg = {};
    cfg.port = pin_config.port;
    cfg.pin.Pin = pin_config.pin;
    cfg.pin.Mode = GPIO_MODE_ANALOG;
    cfg.pin.Pull = GPIO_NOPULL;
    mcu::gpio::AnalogPin input(cfg);

    if (HAL_ADCEx_InjectedConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC injected channel initialization failed");
    }
}


void Module::init_regular_channel(const PinConfig& pin_config, const RegularChannelConfig& channel_config) {
    mcu::gpio::Config cfg = {};
    cfg.port = pin_config.port;
    cfg.pin.Pin = pin_config.pin;
    cfg.pin.Mode = GPIO_MODE_ANALOG;
    cfg.pin.Pull = GPIO_NOPULL;
    mcu::gpio::AnalogPin input(cfg);

    if (channel_config.ranks.size() > 0) {
        for (auto rank : channel_config.ranks) {
            auto config = channel_config;
            config.hal_config.Rank = rank;
            if (HAL_ADC_ConfigChannel(&_handle, &config.hal_config) != HAL_OK) {
                fatal_error("ADC regular channel initialization failed");
            }
        }
    } else {
        auto config = channel_config;
        if (HAL_ADC_ConfigChannel(&_handle, &config.hal_config) != HAL_OK) {
            fatal_error("ADC regular channel initialization failed");
        }
    }
}


void Module::init_injected_internal_channel(InjectedChannelConfig channel_config) {
    if (HAL_ADCEx_InjectedConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC injected channel initialization failed");
    }
}


void Module::init_regular_internal_channel(RegularChannelConfig channel_config) {
    if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC internal channel initialization failed");
    }
}


void Module::init_interrupts(uint32_t interrupt_list, mcu::IrqPriority priority) {
    clear_bit<uint32_t>(_reg->SR, ADC_SR_AWD | ADC_SR_EOC | ADC_SR_JEOC | ADC_SR_JSTRT | ADC_SR_STRT | ADC_SR_OVR);
    set_bit(_reg->CR1, interrupt_list);
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


} // namespace adc


} // namespace mcu


#endif
#endif
