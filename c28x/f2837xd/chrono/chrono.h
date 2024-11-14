#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include <emblib/core.h>
#include <emblib/chrono.h>
#include <emblib/static_vector.h>


namespace mcu {

namespace c28x {

namespace chrono {


inline void delay(emb::chrono::nanoseconds ns) {
    const uint32_t sysclkCycle_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
    const uint32_t delayLoop_ns = 5 * sysclkCycle_ns;
    const uint32_t delayOverhead_ns = 9 * sysclkCycle_ns;

    if (ns.count() < delayLoop_ns + delayOverhead_ns) {
        SysCtl_delay(1);
    } else {
        SysCtl_delay((ns.count() - delayOverhead_ns) / delayLoop_ns);
    }
}


inline void delay(emb::chrono::microseconds us) {
    DEVICE_DELAY_US(us.count());
}


inline void delay(emb::chrono::milliseconds ms) {
    delay(emb::chrono::duration_cast<emb::chrono::microseconds>(ms));
}


class steady_clock {
private:
    static bool _initialized;
    static volatile int64_t _time;
    static const emb::chrono::milliseconds time_step;
private:
    steady_clock();                                     // no constructor
    steady_clock(const steady_clock& other);            // no copy constructor
    steady_clock& operator=(const steady_clock& other); // no copy assignment operator
public:
    static bool initialized() { return _initialized; }
    static emb::chrono::milliseconds now() { return emb::chrono::milliseconds(_time); }
    static emb::chrono::milliseconds step() { return time_step; }
#ifdef CPU1
    static void init();
    static void reset() { _time = 0; }
protected:
    static interrupt void on_interrupt() {
        _time += time_step.count();
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    }
#endif
};


class high_resolution_clock {
private:
    static bool _initialized;
    static uint32_t _period;
    static const int64_t sysclk_period_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
private:
    high_resolution_clock();                                                // no constructor
    high_resolution_clock(const high_resolution_clock& other);              // no copy constructor
    high_resolution_clock& operator=(const high_resolution_clock& other);   // no copy assignment operator
public:
    static void init(emb::chrono::microseconds period);
    static bool initialized() { return _initialized; }
    static uint32_t counter() { return CPUTimer_getTimerCount(CPUTIMER1_BASE); }
    static emb::chrono::nanoseconds now() {	return emb::chrono::nanoseconds((_period - counter()) * sysclk_period_ns); }
    static void start() { CPUTimer_startTimer(CPUTIMER1_BASE); }
    static void stop() { CPUTimer_stopTimer(CPUTIMER1_BASE); }

    static void register_interrupt_handler(void (*handler)(void)) {
        Interrupt_register(INT_TIMER1, handler);
        CPUTimer_enableInterrupt(CPUTIMER1_BASE);
        Interrupt_enable(INT_TIMER1);
    }
private:
    static interrupt void on_interrupt();
};


} // namespace chrono

} // namespace c28x

} // namespace mcu


#endif
