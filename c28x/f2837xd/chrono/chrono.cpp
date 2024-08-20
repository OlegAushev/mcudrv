#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/chrono/chrono.h>


namespace mcu {


namespace chrono {


bool steady_clock::_initialized = false;
volatile int64_t steady_clock::_time = 0;
const emb::chrono::milliseconds steady_clock::time_step(1);


void steady_clock::init() {
    _time = 0;

    Interrupt_register(INT_TIMER0, steady_clock::on_interrupt);

    CPUTimer_stopTimer(CPUTIMER0_BASE);             // Make sure timer is stopped
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF); // Initialize timer period to maximum
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);       // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);    // Reload counter register with period value

    uint32_t tmp = (uint32_t)((mcu::sysclk_freq() / 1000) * time_step.count());
    CPUTimer_setPeriod(CPUTIMER0_BASE, tmp - 1);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}


uint32_t high_resolution_clock::_period;


void high_resolution_clock::init(emb::chrono::microseconds period) {
    CPUTimer_stopTimer(CPUTIMER1_BASE);             // Make sure timer is stopped
    CPUTimer_setPeriod(CPUTIMER1_BASE, 0xFFFFFFFF); // Initialize timer period to maximum
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 0);       // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);    // Reload counter register with period value

    _period = (uint32_t)(mcu::sysclk_freq() / 1000000) * period.count() - 1;
    CPUTimer_setPeriod(CPUTIMER1_BASE, _period);
    CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
}


} // namespace chrono


} // namespace mcu


#endif
