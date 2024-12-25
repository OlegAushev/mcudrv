#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_misc.h>

#include <mcudrv/apm32/f4/apm32f4_base.h>

#include <array>
#include <chrono>
#include <emblib/static_vector.hpp>

extern "C" void SysTick_Handler();

namespace mcu {
namespace apm32 {
namespace chrono {

class steady_clock {
    friend void ::SysTick_Handler();
private:
    static inline bool _initialized{false};
    static inline volatile int64_t _time{0};
    static constexpr std::chrono::milliseconds time_step{1};
public:
    steady_clock() = delete;
    static void init();
    static bool initialized() { return _initialized; }
    static std::chrono::milliseconds now() {
        return std::chrono::milliseconds(_time);
    }
    static std::chrono::milliseconds step() { return time_step; }

    static void delay(std::chrono::milliseconds delay) {
        auto start = now();
        while ((now() - start) <= delay) { /* wait */
        }
    }
protected:
    static void on_interrupt() { _time = _time + time_step.count(); }
};

class high_resolution_clock {
private:
    static inline bool _initialized{false};
    static inline uint32_t _ticks_usec{1};
public:
    high_resolution_clock() = delete;
    static void init();
    static bool initialized() { return _initialized; }

    static std::chrono::microseconds now() {
        std::chrono::microseconds usec{(SysTick->LOAD - SysTick->VAL) /
                                       _ticks_usec};
        return steady_clock::now() + usec;
    }

    static void delay(std::chrono::microseconds delay) {
        auto start = now();
        while ((now() - start) <= delay) {
            // wait
        }
    }
};

} // namespace chrono
} // namespace apm32
} // namespace mcu

#endif
#endif
