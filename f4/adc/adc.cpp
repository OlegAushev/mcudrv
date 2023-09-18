#ifdef STM32F4xx

#include <mculib_stm32/f4/adc/adc.h>


namespace mcu {

namespace adc {


Module::Module(Peripheral peripheral, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    _enable_clk(peripheral);
    _handle.Instance = impl::adc_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_config;
    
    _reg = _handle.Instance;

    if (HAL_ADC_Init(&_handle) != HAL_OK) {
        fatal_error("ADC module initialization failed");
    }

    set_bit(_reg->CR2, ADC_CR2_ADON);
    auto counter = (ADC_STAB_DELAY_US * (core_clk_freq() / 1000000));
    while(counter != 0)
    {
        --counter;
    }
}


void Module::add_injected_channel(PinConfig pin_config, InjectedChannelConfig channel_config) {
    mcu::gpio::Config cfg = {};
    cfg.port = pin_config.port;
    cfg.pin.Pin = pin_config.pin;
    cfg.pin.Mode = GPIO_MODE_ANALOG;
    cfg.pin.Pull = GPIO_NOPULL;
    mcu::gpio::Input input(cfg);

    if (HAL_ADCEx_InjectedConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC injected channel initialization failed");
    }
}


void Module::add_regular_channel(PinConfig pin_config, RegularChannelConfig channel_config) {
    mcu::gpio::Config cfg = {};
    cfg.port = pin_config.port;
    cfg.pin.Pin = pin_config.pin;
    cfg.pin.Mode = GPIO_MODE_ANALOG;
    cfg.pin.Pull = GPIO_NOPULL;
    mcu::gpio::Input input(cfg);

    if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC regular channel initialization failed");
    }
}


void Module::add_injected_internal_channel(InjectedChannelConfig channel_config) {
    if (HAL_ADCEx_InjectedConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC injected channel initialization failed");
    }
}


void Module::add_regular_internal_channel(RegularChannelConfig channel_config) {
    if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC internal channel initialization failed");
    }
}


void Module::_enable_clk(Peripheral peripheral) {
    auto adc_idx = std::to_underlying(peripheral);
    if (_clk_enabled[adc_idx]) {
        return;
    }

    impl::adc_clk_enable_funcs[adc_idx]();
    _clk_enabled[adc_idx] = true;
}


void Module::init_interrupts() {
    switch (_peripheral) {
    case Peripheral::adc1:
        _init_adc1_interrupts();
        break;
    case Peripheral::adc2:
        _init_adc2_interrupts();
        break;
    case Peripheral::adc3:
        _init_adc3_interrupts();
        break;
    }
}


__attribute__((weak)) void Module::_init_adc1_interrupts() {}
__attribute__((weak)) void Module::_init_adc2_interrupts() {}
__attribute__((weak)) void Module::_init_adc3_interrupts() {}


} // namespace adc

} // namespace mcu


#endif
