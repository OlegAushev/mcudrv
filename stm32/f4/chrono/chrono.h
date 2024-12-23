#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include <mcudrv/stm32/f4/stm32_f4_base.h>
#include <mcudrv/stm32/f4/system/system.h>
#include <emblib/core.hpp>
#include <emblib/static_vector.hpp>
#include <chrono>


extern "C" void SysTick_Handler();


namespace mcu {
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
    static std::chrono::milliseconds now() { return std::chrono::milliseconds(_time); }
    static std::chrono::milliseconds step() { return time_step; }

    static void delay(std::chrono::milliseconds delay) {
        auto start = now();
        while ((now() - start) <= delay) { /* wait */ }
    }
protected:
    static void on_interrupt() { _time += time_step.count(); }
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
        std::chrono::microseconds usec{(SysTick->LOAD - SysTick->VAL) / _ticks_usec};
        return steady_clock::now() + usec;
    }
    
    static void delay(std::chrono::microseconds delay) {
        auto start = now();
        while ((now() - start) <= delay) { /* wait */ }
    }
};


} // namespace chrono
} // namespace mcu


#endif
#endif
