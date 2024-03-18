#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/timers/advanced/advanced.h>


namespace mcu {


namespace timers {


namespace advanced {


impl::AbstractTimer::AbstractTimer(Peripheral peripheral, OpMode mode)
        : emb::interrupt_invoker_array<AbstractTimer, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _reg(impl::instances[std::to_underlying(peripheral)])
        , _mode(mode)
{
    _enable_clk(peripheral);   
}


void impl::AbstractTimer::_enable_clk(Peripheral peripheral) {
    auto timer_idx = std::to_underlying(peripheral);
    if (_clk_enabled[timer_idx]) {
        return;
    }

    impl::clk_enable_funcs[timer_idx]();
    _clk_enabled[timer_idx] = true;
}


} // namespace adv


} // namespace timers


} // namepsace mcu


#endif
#endif
