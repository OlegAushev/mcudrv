#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/gen1/input_capture.h>


namespace mcu {
namespace tim {
namespace gen1 {


InputCaptureTimer::InputCaptureTimer(Peripheral peripheral, const InputCaptureConfig& config)
        : impl::AbstractTimer(peripheral, OpMode::input_capture)
{
    _handle.Init = config.hal_base_config;

    if (HAL_TIM_IC_Init(&_handle) != HAL_OK) {
        fatal_error();
    }
}


void InputCaptureTimer::initialize_channel(Channel channel, ChPin* pin_ch, const InputCaptureChannelConfig& config) {
    if (pin_ch == nullptr) {
        fatal_error();
    }
    
    auto cfg = config;
    if (HAL_TIM_IC_ConfigChannel(&_handle, &cfg.hal_ic_config, std::to_underlying(channel)) != HAL_OK) {
        fatal_error();
    }

    set_bit(_reg->CCER, uint32_t(TIM_CCx_ENABLE) << std::to_underlying(channel));
}


} // namespace general
} // namespace timers
} // namespace mcu


#endif
#endif
