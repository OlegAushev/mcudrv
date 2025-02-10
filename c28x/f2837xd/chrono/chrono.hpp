#pragma once

#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/system/system.hpp>
#include <emblib/core.hpp>
#include <emblib/chrono.hpp>
#include <emblib/static_vector.hpp>

namespace mcu {
namespace c28x {
namespace chrono {

inline void delay(emb::chrono::nanoseconds ns) {
    const uint32_t sysclk_cycle_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
    const uint32_t delay_loop_ns = 5 * sysclk_cycle_ns;
    const uint32_t delay_overhead_ns = 9 * sysclk_cycle_ns;

    if (ns.count() < delay_loop_ns + delay_overhead_ns) {
        SysCtl_delay(1);
    } else {
        SysCtl_delay((ns.count() - delay_overhead_ns) / delay_loop_ns);
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
    static bool initialized_;
    static volatile int64_t time_;
    static const emb::chrono::milliseconds time_step;
private:
    steady_clock();                                     // delete
    steady_clock(const steady_clock& other);            // delete
    steady_clock& operator=(const steady_clock& other); // delete
public:
    static bool initialized() { return initialized_; }

    static emb::chrono::milliseconds now() {
        return emb::chrono::milliseconds(time_);
    }

    static emb::chrono::milliseconds step() { return time_step; }
#ifdef CPU1
    static void init();
    static void reset() { time_ = 0; }
protected:
    static interrupt void on_interrupt() {
        time_ += time_step.count();
        Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    }
#endif
};

class high_resolution_clock {
private:
    static bool initialized_;
    static uint32_t period_;
    static const int64_t sysclk_period_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
private:
    high_resolution_clock();                                    // delete
    high_resolution_clock(const high_resolution_clock& other);  // delete
    high_resolution_clock& operator=(const high_resolution_clock& other);
public:
    static void init(emb::chrono::microseconds period);
    static bool initialized() { return initialized_; }

    static uint32_t counter() { return CPUTimer_getTimerCount(CPUTIMER1_BASE); }

    static emb::chrono::nanoseconds now() {
        return emb::chrono::nanoseconds((period_ - counter()) *
                                        sysclk_period_ns);
    }

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
