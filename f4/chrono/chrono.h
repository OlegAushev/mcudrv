#pragma once


#ifdef STM32F4xx

#include "../mcu_def.h"
#include "../system/system.h"
#include <emblib_stm32/core.h>
#include <emblib_stm32/staticvector.h>
#include <array>
#include <chrono>


extern "C" void SysTick_Handler();


namespace mcu {

namespace chrono {

enum class TaskStatus {
    success,
    fail
};


class system_clock : private emb::noncopyable, public emb::monostate<system_clock> {
    friend void ::SysTick_Handler();
public:
    system_clock() = delete;
    static void init();
private:
    static inline volatile int64_t _time = 0;
    static constexpr std::chrono::milliseconds time_step = std::chrono::milliseconds(1);
    static constexpr int_fast8_t task_count_max = 4;

/* periodic tasks */
private:
    struct Task
    {
        std::chrono::milliseconds period;
        std::chrono::milliseconds timepoint;
        TaskStatus (*func)(int);
    };
    static TaskStatus empty_task(int) { return TaskStatus::success; }
    static inline emb::static_vector<Task, task_count_max> _tasks;
public:
    static void add_task(TaskStatus (*func)(int), std::chrono::milliseconds period)
    {
        Task task = {period, now(), func};
        _tasks.push_back(task);
    }

    static void set_task_period(int index, std::chrono::milliseconds period)
    {
        if (index < _tasks.size()) {
            _tasks[index].period = period;
        }
    }
private:
    static inline std::chrono::milliseconds _delayed_task_start = std::chrono::milliseconds(0);
    static inline std::chrono::milliseconds _delayed_task_delay = std::chrono::milliseconds(0);
    static void empty_delayed_task() {}
    static inline void (*_delayed_task)() = empty_delayed_task;
public:
    static void register_delayed_task(void (*task)(), std::chrono::milliseconds delay)
    {
        _delayed_task = task;
        _delayed_task_delay = delay;
        _delayed_task_start = now();
    }

public:
    static std::chrono::milliseconds now() { return std::chrono::milliseconds(_time); }
    static std::chrono::milliseconds step() { return time_step; }

    static void reset() {
        _time = 0;
        for (auto i = 0; i < _tasks.size(); ++i) {
            _tasks[i].timepoint = now();
        }
    }

    static void run_tasks(); 

protected:
    static void on_interrupt() { _time += time_step.count(); }
};

} // namespace chrono

} // namespace mcu

#endif
