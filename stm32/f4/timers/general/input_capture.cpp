#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/timers/general/input_capture.h>


namespace mcu {


namespace timers {


namespace general {


InputCaptureTimer::InputCaptureTimer(Peripheral peripheral, const InputCaptureConfig& config)
        : impl::AbstractTimer(peripheral, OpMode::input_capture)
{
    
}



} // namespace general


} // namespace timers


} // namespace mcu


#endif
#endif
