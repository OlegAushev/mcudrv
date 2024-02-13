#include "mcudrv/stm32/f4/gpio/gpio.h"
#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/adc/adc.h>


namespace mcu {


namespace adc {


Module::Module(Peripheral peripheral, const Config& config, dma::Stream* dma)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _reg(impl::adc_instances[std::to_underlying(peripheral)])
        , _reg_common(impl::adc_common_instances[std::to_underlying(peripheral)]) {
    _enable_clk(_peripheral);
    _handle.Instance = impl::adc_instances[std::to_underlying(_peripheral)];
    _handle.Init = config.hal_config;

    if (HAL_ADC_Init(&_handle) != HAL_OK) {
        fatal_error("ADC module initialization failed");
    }

    _calibrate();

    set_bit<uint32_t>(_reg->CR, ADC_CR_ADEN);
    while (bit_is_clear<uint32_t>(_reg->ISR, ADC_ISR_ADRDY)) {
        // wait
    }

    if (dma) {
        //modify_reg<uint32_t>(_reg->CFGR, ADC_CFGR_DMNGT, config.hal_config.ConversionDataManagement);
        write_reg(dma->stream_reg()->PAR, uint32_t(&(_reg->DR)));
    }
}


void Module::_calibrate() {
    /* Run the ADC calibration in single-ended mode */
    if (HAL_ADCEx_Calibration_Start(&_handle, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK) {
        fatal_error("ADC module calibration failed");
    }
}


void Module::initialize_regular_channel(const PinConfig& pin_config, const RegularChannelConfig& channel_config) {
    mcu::gpio::Config cfg{};
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


// void Module::initialize_regular_channel(PinConfig pin_config, ChannelConfig channel_config) {
//     mcu::gpio::Config cfg = {};
//     cfg.port = pin_config.port;
//     cfg.pin.Pin = pin_config.pin;
//     cfg.pin.Mode = GPIO_MODE_ANALOG;
//     cfg.pin.Pull = GPIO_NOPULL;
//     mcu::gpio::AnalogPin input(cfg);

//     if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
//         fatal_error("ADC regular channel initialization failed");
//     }
// }


// void Module::initialize_internal_channel(ChannelConfig channel_config) {
//     if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
//         fatal_error("ADC internal channel initialization failed");
//     }
// }


void Module::initialize_injected_internal_channel(InjectedChannelConfig channel_config) {
    if (HAL_ADCEx_InjectedConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC injected channel initialization failed");
    }
}


void Module::initialize_regular_internal_channel(RegularChannelConfig channel_config) {
    if (HAL_ADC_ConfigChannel(&_handle, &channel_config.hal_config) != HAL_OK) {
        fatal_error("ADC internal channel initialization failed");
    }
}


void Module::_enable_clk(Peripheral peripheral) {
    switch (peripheral) {
    case Peripheral::adc1:
    case Peripheral::adc2:
        if (_clk_enabled[0]) return;
        __HAL_RCC_ADC12_CLK_ENABLE();
        _clk_enabled[0] = true;
        break;
    case Peripheral::adc3:
        if (_clk_enabled[1]) return;
        __HAL_RCC_ADC3_CLK_ENABLE();
        _clk_enabled[1] = true;
        break;
    default:
        fatal_error("invalid ADC module");
        break;
    }
}


} // namespace adc


} // namespace mcu


// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* handle) {
//     using namespace mcu::adc;
//     Module::instance(impl::to_peripheral(handle->Instance))->on_half_completed();
// }


// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* handle) {
//     using namespace mcu::adc;
//     Module::instance(impl::to_peripheral(handle->Instance))->on_completed();
// }


// void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *handle) {
//     using namespace mcu::adc;
//     Module::instance(impl::to_peripheral(handle->Instance))->on_error();
// }


#endif
#endif
