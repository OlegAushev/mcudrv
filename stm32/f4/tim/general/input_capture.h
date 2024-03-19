#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/general/general.h>


namespace mcu {


namespace tim {
    
    
namespace general {


class InputCaptureTimer : public impl::AbstractTimer {
private:

public:
    InputCaptureTimer(Peripheral peripheral, const InputCaptureConfig& config);
};


} // namespace general


} // namespace timers


} // namepsace mcu


#endif
#endif
