#pragma once


#include "../system/system.h"
#include <emblib/core.h>
#include <emblib/chrono.h>
#include <emblib/staticvector.h>


namespace mcu {

namespace chrono {

SCOPED_ENUM_DECLARE_BEGIN(TaskStatus) {
    success = 0,
    fail = 1
} SCOPED_ENUM_DECLARE_END(TaskStatus)


class system_clock : public emb::monostate<system_clock> {
private:
    static volatile int64_t _time;
    static const emb::chrono::milliseconds time_step;
    static const int task_count_max = 4;
private:
    struct Task {
        emb::chrono::milliseconds period;
        emb::chrono::milliseconds timepoint;
        TaskStatus (*func)(int);
    };
    static TaskStatus empty_task() { return TaskStatus::success; }
    static emb::static_vector<Task, task_count_max> _tasks;
public:
    static void add_task(TaskStatus (*func)(int), emb::chrono::milliseconds period) {
        Task task = {period, now(), func};
        _tasks.push_back(task);
    }

    static void set_task_period(int index, emb::chrono::milliseconds period) {
        if (index < _tasks.size()) {
            _tasks[index].period = period;
        }
    }
private:
    static emb::chrono::milliseconds _delayed_task_start;
    static emb::chrono::milliseconds _delayed_task_delay;
    static void (*_delayed_task)();
    static void empty_delayed_task() {}
public:
    static void register_delayed_task(void (*task)(), emb::chrono::milliseconds delay) {
        _delayed_task = task;
        _delayed_task_delay = delay;
        _delayed_task_start = now();
    }
private:
    system_clock();                                     // no constructor
    system_clock(const system_clock& other);            // no copy constructor
    system_clock& operator=(const system_clock& other); // no copy assignment operator
public:
    static void init();
    static emb::chrono::milliseconds now() { return emb::chrono::milliseconds(_time); }
    static emb::chrono::milliseconds step() { return time_step; }
    static void run_tasks();

    static void reset() {
        _time = 0;
        for (int i = 0; i < _tasks.size(); ++i) {
            _tasks[i].timepoint = now();
        }
    }
protected:
    static interrupt void on_interrupt();
};


class high_resolution_clock : public emb::monostate<high_resolution_clock> {
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
            , _start(system_clock::now()) {}

    bool expired() {
        if (_timeout.count() < 0) {
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

