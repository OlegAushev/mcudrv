#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/timers/general_purpose/base.h>


namespace mcu {


namespace timers {
    
    
namespace gp {


class InputCaptureTimer : public impl::AbstractTimer {
private:

public:
    InputCaptureTimer(Peripheral peripheral, const InputCaptureConfig& config);
};


} // namespace gp


} // namespace timers


} // namepsace mcu


#endif
#endif
