#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include <emblib/core.h>
#include <emblib/chrono.h>
#include <emblib/static_vector.h>


namespace mcu {


namespace chrono {


SCOPED_ENUM_DECLARE_BEGIN(TaskStatus) {
    success = 0,
    fail = 1
} SCOPED_ENUM_DECLARE_END(TaskStatus)


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
    static void init();
    static bool initialized() { return _initialized; }
    static emb::chrono::milliseconds now() { return emb::chrono::milliseconds(_time); }
    static emb::chrono::milliseconds step() { return time_step; }
    static void reset() { _time = 0; }
protected:
    static interrupt void on_interrupt() {
        _time += time_step.count();
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    }
};


class high_resolution_clock {
private:
    static uint32_t _period;
    static const int64_t sysclk_period_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
public:
    static void init(emb::chrono::microseconds period);
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


class Timeout {
private:
    const emb::chrono::milliseconds _timeout;
    emb::chrono::milliseconds _start;
public:
    Timeout(emb::chrono::milliseconds timeout = emb::chrono::milliseconds(-1))
            : _timeout(timeout)
            , _start(steady_clock::now()) {}

    bool expired() {
        if (_timeout.count() < 0) {
            return false;
        }
        if ((steady_clock::now() - _start) > _timeout) {
            return true;
        }
        return false;
    }

    void reset() { _start = steady_clock::now(); }
};


} // namespace chrono


} // namespace mcu


#endif
