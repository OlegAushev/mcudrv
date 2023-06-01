#include <mculib_stm32/h7/adc/adc.h>


namespace mcu {

namespace adc {


Module::Module(Peripheral peripheral, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral) {
    enable_clk();
    _handle.Instance = impl::adc_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_init;

    if (HAL_ADC_Init(&_handle) != HAL_OK) {
        fatal_error("ADC module initialization failed");
    }
}


void Module::calibrate() {
    /* Run the ADC calibration in single-ended mode */
    if (HAL_ADCEx_Calibration_Start(&_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        fatal_error("ADC calibration failed");
    }
}


void Module::add_regular_channel(PinConfig pin_config, ChannelConfig channel_config) {
    mcu::gpio::Config cfg = {};
    cfg.port = pin_config.port;
    cfg.pin.Pin = pin_config.pin;
    cfg.pin.Mode = GPIO_MODE_ANALOG;
    cfg.pin.Pull = GPIO_NOPULL;
    mcu::gpio::Input input(cfg);

    if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_init) != HAL_OK) {
        fatal_error("ADC regular channel initialization failed");
    }
}


void Module::add_internal_channel(ChannelConfig channel_config) {
    if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_init) != HAL_OK) {
        fatal_error("ADC internal channel initialization failed");
    }
}

} // namespace adc

} // namespace mcu


void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* handle) {
    using namespace mcu::adc;
    Module::instance(impl::to_peripheral(handle->Instance))->on_half_completed();
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* handle) {
    using namespace mcu::adc;
    Module::instance(impl::to_peripheral(handle->Instance))->on_completed();
}


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle) {
    using namespace mcu::adc;
    Module::instance(impl::to_peripheral(handle->Instance))->on_error();
}

