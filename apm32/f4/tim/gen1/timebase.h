#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/tim/gen1/base.h>


namespace mcu {
namespace tim {
namespace gen1 {


struct TimebaseConfig {
    float freq;
    bool arr_preload;
    TMR_BaseConfig_T hal_base_config;
};


class TimebaseTimer : public impl::AbstractTimer {
private:
    float _freq{0};
public:
    TimebaseTimer(Peripheral peripheral, const TimebaseConfig& config);

    static TimebaseTimer* instance(Peripheral peripheral) {
        assert(impl::AbstractTimer::instance(std::to_underlying(peripheral))->mode() == OpMode::timebase);
        return static_cast<TimebaseTimer*>(impl::AbstractTimer::instance(std::to_underlying(peripheral)));
    }

    float freq() const { return _freq; }

    void init_interrupts(IrqPriority priority) {
        _reg->DIEN_B.UIEN = 1;
        set_irq_priority(impl::irq_nums[std::to_underlying(_peripheral)], priority);
    }

    void enable_interrupts() {
        _reg->STS_B.UIFLG = 0;
        clear_pending_irq(impl::irq_nums[std::to_underlying(_peripheral)]);
        enable_irq(impl::irq_nums[std::to_underlying(_peripheral)]);
    }

    void disable_interrupts() {
        disable_irq(impl::irq_nums[std::to_underlying(_peripheral)]);
    }

    void ack_interrupt() {
        _reg->STS_B.UIFLG = 0;
    }
};



} // namespace gen1
} // namespace tim
} // namespace mcu


#endif
#endif
