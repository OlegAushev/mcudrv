#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/chrono/chrono.hpp>


namespace mcu {

namespace c28x {

namespace chrono {


bool steady_clock::_initialized __attribute__((section("shared_mcu_chrono"), retain)) = false;
volatile int64_t steady_clock::_time __attribute__((section("shared_mcu_chrono"), retain)) = 0;
const emb::chrono::milliseconds steady_clock::time_step __attribute__((section("shared_mcu_chrono"), retain)) = emb::chrono::milliseconds(1);


#ifdef CPU1
void steady_clock::init() {
    _time = 0;

    Interrupt_register(INT_TIMER0, steady_clock::on_interrupt);

    CPUTimer_stopTimer(CPUTIMER0_BASE);             // Make sure timer is stopped
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF); // Initialize timer period to maximum
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);       // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);    // Reload counter register with period value

    uint32_t tmp = (uint32_t)((mcu::c28x::sysclk_freq() / 1000) * time_step.count());
    CPUTimer_setPeriod(CPUTIMER0_BASE, tmp - 1);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);

    _initialized = true;
}
#endif


bool high_resolution_clock::_initialized = false;
uint32_t high_resolution_clock::_period;


void high_resolution_clock::init(emb::chrono::microseconds period) {
    CPUTimer_stopTimer(CPUTIMER1_BASE);             // Make sure timer is stopped
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF); // Initialize timer period to maximum
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);       // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);    // Reload counter register with period value

    _period = (uint32_t)(mcu::c28x::sysclk_freq() / 1000000) * period.count() - 1;
    CPUTimer_setPeriod(CPUTIMER1_BASE, _period);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    _initialized = true;
}


} // namespace chrono

} // namespace c28x

} // namespace mcu


#endif
