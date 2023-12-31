#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include "../system/system.h"
#include <emblib/core.h>
#include <emblib/static_vector.h>
#include <array>
#include <chrono>


extern "C" void SysTick_Handler();


namespace mcu {


namespace chrono {


enum class TaskStatus {
    success,
    fail
};


class system_clock {
    friend void ::SysTick_Handler();
public:
    system_clock() = delete;
    static void init();
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
            , _start(mcu::chrono::system_clock::now())
    {}

    bool expired() const {
        if (_timeout.count() <= 0) {
            return false;
        }
        if ((system_clock::now() - _start) > _timeout) {
            return true;
        }
        return false;
    }

    void reset() { _start = system_clock::now(); }
};


} // namespace chrono


} // namespace mcu


#endif
#endif
