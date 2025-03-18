#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/chrono/chrono.hpp>

namespace mcu {
namespace c28x {
namespace chrono {

bool steady_clock::initialized_ __attribute__((section("mcu_chrono"), retain)) = false;
volatile int64_t steady_clock::time_ __attribute__((section("mcu_chrono"), retain)) = 0;
const emb::chrono::milliseconds steady_clock::time_step __attribute__((section("mcu_chrono"), retain)) = emb::chrono::milliseconds(1);

#ifdef CPU1
void steady_clock::init() {
    time_ = 0;

    Interrupt_register(INT_TIMER0, steady_clock::on_interrupt);

    CPUTimer_stopTimer(CPUTIMER0_BASE);
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    const uint32_t sysclk_freq_kHz = mcu::c28x::sysclk_freq() / uint32_t(1000);
    const uint32_t period = sysclk_freq_kHz * uint32_t(time_step.count()) - 1;
    CPUTimer_setPeriod(CPUTIMER0_BASE, period);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);

    CPUTimer_startTimer(CPUTIMER0_BASE);

    initialized_ = true;
}
#endif

bool high_resolution_clock::initialized_ = false;
uint32_t high_resolution_clock::period_;

void high_resolution_clock::init(emb::chrono::microseconds period) {
    CPUTimer_stopTimer(CPUTIMER1_BASE);
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF);
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);

    const uint32_t sysclk_freq_MHz = mcu::c28x::sysclk_freq() /
                                     uint32_t(1000000);
    period_ = sysclk_freq_MHz * uint32_t(period.count()) - 1;
    CPUTimer_setPeriod(CPUTIMER1_BASE, period_);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    initialized_ = true;
}

} // namespace chrono
} // namespace c28x
} // namespace mcu

#endif
