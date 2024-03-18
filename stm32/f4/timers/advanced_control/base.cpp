#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/timers/advanced_control/base.h>


namespace mcu {


namespace timers {


namespace adv {


impl::AbstractTimer::AbstractTimer(Peripheral peripheral, OpMode mode)
        : emb::interrupt_invoker_array<AbstractTimer, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _mode(mode)
{
    _enable_clk(peripheral);
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


} // namespace adv


} // namespace timers


} // namespace mcu


#endif
#endif
