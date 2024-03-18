#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/timers/timersdef.h>


namespace mcu {


namespace timers {


namespace advanced {


constexpr size_t peripheral_count = 2;
enum class Peripheral : unsigned int {
    tim1,
    tim8
};


namespace impl {


inline const std::array<TMR_T*, peripheral_count> instances = {TMR1, TMR8};


inline Peripheral to_peripheral(const TMR_T* instance) {
    return static_cast<Peripheral>(std::distance(instances.begin(),
                                   std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR1); },
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_TMR8); },
};


inline constexpr std::array<IRQn_Type, peripheral_count> up_irq_nums = {TMR1_UP_TMR10_IRQn, TMR8_UP_TMR13_IRQn};
inline constexpr std::array<IRQn_Type, peripheral_count> brk_irq_nums = {TMR1_BRK_TMR9_IRQn, TMR8_BRK_TMR12_IRQn};


class AbstractTimer : public emb::interrupt_invoker_array<AbstractTimer, peripheral_count>, public emb::noncopyable {
protected:
    const Peripheral _peripheral;
    TMR_T* const _reg;
    const OpMode _mode;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    AbstractTimer(Peripheral peripheral, OpMode mode);
    Peripheral peripheral() const { return _peripheral; }
    TMR_T* reg() { return _reg; }
    OpMode mode() const { return _mode; }

    void enable() {
        _reg->CTRL1_B.CNTEN = 1;
    }

    void disable() {
        _reg->CTRL1_B.CNTEN = 0;
    }
private:
    static void _enable_clk(Peripheral peripheral);
};


} // namespace impl


} // namespace adv


} // namespace timers


} // namepsace mcu


#endif
#endif
