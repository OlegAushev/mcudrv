#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.h>
#include <apm32f4xx_misc.h>
#include <emblib/core.h>
#include <emblib/static_vector.h>
#include <array>
#include <chrono>


extern "C" void SysTick_Handler();


namespace mcu {
namespace chrono {


class steady_clock {
    friend void ::SysTick_Handler();
public:
    steady_clock() = delete;
    static void initialize();
private:
    static inline volatile int64_t _time{0};
    static constexpr std::chrono::milliseconds time_step{1};
public:
    static std::chrono::milliseconds now() { return std::chrono::milliseconds(_time); }
    static std::chrono::milliseconds step() { return time_step; }
protected:
    static void on_interrupt() { _time += time_step.count(); }
};


} // namespace chrono
} // namespace mcu


#endif
#endif
