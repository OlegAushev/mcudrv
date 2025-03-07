#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/stm32_h7_base.h>
#include <mcudrv/stm32/h7/system/system.h>
#include <emblib/core.hpp>
#include <emblib/static_vector.hpp>
#include <array>
#include <chrono>


extern "C" void SysTick_Handler();


namespace mcu {


namespace chrono {


enum class TaskStatus {
    success,
    fail
};


class steady_clock {
    friend void ::SysTick_Handler();
public:
    steady_clock() = delete;
    static void initialize();
private:
    static inline volatile int64_t _time{0};
    static constexpr std::chrono::milliseconds time_step{1};
    static constexpr int_fast8_t task_count_max{4};
public:
    static std::chrono::milliseconds now() { return std::chrono::milliseconds(_time); }
    static std::chrono::milliseconds step() { return time_step; }
protected:
    static void on_interrupt() { _time += time_step.count(); }
};


class Timeout {
private:
    const std::chrono::milliseconds _timeout;
    std::chrono::milliseconds _start;
public:
    Timeout(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))
            : _timeout(timeout)
            , _start(mcu::chrono::steady_clock::now())
    {}

    bool expired() const {
        if (_timeout.count() <= 0) {
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
#endif
