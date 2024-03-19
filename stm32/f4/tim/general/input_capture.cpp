#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/tim/general/input_capture.h>


namespace mcu {


namespace tim {


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
