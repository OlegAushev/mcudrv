#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/timers/general_purpose_timer.h>


namespace mcu {


namespace timers {


namespace gp {


impl::AbstractTimer::AbstractTimer(Peripheral peripheral)
        : emb::interrupt_invoker_array<AbstractTimer, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
{
    _reg = impl::instances[std::to_underlying(_peripheral)];
    _handle.Instance = _reg;
}


void impl::AbstractTimer::_enable_clk(Peripheral peripheral) {
    auto timer_idx = std::to_underlying(peripheral);
    if (_clk_enabled[timer_idx]) {
        return;
    }

    impl::clk_enable_funcs[timer_idx]();
    _clk_enabled[timer_idx] = true;
}



}


} // namespace timers


} // namespace mcu


#endif
#endif
