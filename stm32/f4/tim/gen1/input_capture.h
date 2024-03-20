#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/gen1/base.h>
#include <initializer_list>


namespace mcu {
namespace tim {
namespace gen1 {


struct InputCaptureConfig {
    TIM_Base_InitTypeDef hal_base_config;
};


struct InputCaptureChannelConfig {
    TIM_IC_InitTypeDef hal_ic_config;
};


class InputCaptureTimer : public impl::AbstractTimer {
private:

public:
    InputCaptureTimer(Peripheral peripheral, const InputCaptureConfig& config);
    void initialize_channel(Channel channel, ChPin* pin_ch, const InputCaptureChannelConfig& config);
    
    uint32_t read_captured(Channel channel) {
        switch (channel) {
        case Channel::channel1:
            return read_reg(_reg->CCR1); 
        case Channel::channel2:
            return read_reg(_reg->CCR2); 
        case Channel::channel3:
            return read_reg(_reg->CCR3); 
        case Channel::channel4:
            return read_reg(_reg->CCR4); 
        }
    }
};


} // namespace general
} // namespace timers
} // namepsace mcu


#endif
#endif
