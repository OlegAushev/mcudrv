#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/chrono/chrono.h>


namespace mcu {


namespace chrono {


volatile int64_t steady_clock::_time;
const emb::chrono::milliseconds steady_clock::time_step(1);

emb::static_vector<steady_clock::Task, steady_clock::task_count_max> steady_clock::_tasks;

emb::chrono::milliseconds steady_clock::_delayed_task_start;
emb::chrono::milliseconds steady_clock::_delayed_task_delay;
void (*steady_clock::_delayed_task)();


void steady_clock::initialize() {
    _time = 0;

    _delayed_task_start = emb::chrono::milliseconds(0);
    _delayed_task_delay = emb::chrono::milliseconds(-1);

    Interrupt_register(INT_TIMER0, steady_clock::on_interrupt);

    CPUTimer_stopTimer(CPUTIMER0_BASE);             // Make sure timer is stopped
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF); // Initialize timer period to maximum
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);       // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);    // Reload counter register with period value

    uint32_t tmp = (uint32_t)((mcu::sysclk_freq() / 1000) * time_step.count());
    CPUTimer_setPeriod(CPUTIMER0_BASE, tmp - 1);
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    _delayed_task = empty_delayed_task;

    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
    Interrupt_enable(INT_TIMER0);
    CPUTimer_startTimer(CPUTIMER0_BASE);
}


void steady_clock::run_tasks() {
    for (size_t i = 0; i < _tasks.size(); ++i) {
        if (now() >= (_tasks[i].timepoint + _tasks[i].period)) {
            if (_tasks[i].func(i) == TaskStatus::success) {
                _tasks[i].timepoint = now();
            }
        }

    }

    if (_delayed_task_delay.count() >= 0) {
        if (now() >= (_delayed_task_start + _delayed_task_delay)) {
            _delayed_task_delay = emb::chrono::milliseconds(-1);
            _delayed_task();
        }
    }
}


interrupt void steady_clock::on_interrupt() {
    _time += time_step.count();
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


uint32_t high_resolution_clock::_period;


void high_resolution_clock::initialize(emb::chrono::microseconds period) {
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
