#pragma once


#include <c28x_emb/emb_core.h>
#include <c28x_emb/emb_staticvector.h>
#include <c28x_mcu/f2837xd/system/mcu_system.h>
#include "driverlib.h"
#include "device.h"


namespace mcu {

namespace chrono {

SCOPED_ENUM_DECLARE_BEGIN(TaskStatus)
{
	success = 0,
	fail = 1
}
SCOPED_ENUM_DECLARE_END(TaskStatus)


class system_clock : public emb::monostate<system_clock>
{
private:
	static volatile uint64_t _time;
	static const uint32_t time_step = 1;
	static const size_t task_count_max = 4;
private:
	struct Task
	{
		uint64_t period;
		uint64_t timepoint;
		TaskStatus (*func)(size_t);
	};
	static TaskStatus empty_task() { return TaskStatus::success; }
	static emb::StaticVector<Task, task_count_max> _tasks;
public:
	static void register_task(TaskStatus (*func)(size_t), uint64_t period)
	{
		Task task = {period, now(), func};
		_tasks.push_back(task);
	}

	static void set_task_period(size_t index, uint64_t period)
	{
		if (index < _tasks.size())
		{
			_tasks[index].period = period;
		}
	}
private:
	static uint64_t _delayed_task_start;
	static uint64_t _delayed_task_delay;
	static void (*_delayed_task)();
	static void empty_delayed_task() {}
public:
	static void register_delayed_task(void (*task)(), uint64_t delay)
	{
		_delayed_task = task;
		_delayed_task_delay = delay;
		_delayed_task_start = now();
	}
private:
	system_clock();						// no constructor
	system_clock(const system_clock& other);			// no copy constructor
	system_clock& operator=(const system_clock& other);	// no copy assignment operator
public:
	static void init();
	static uint64_t now() { return _time; }
	static uint32_t step() { return time_step; }
	static void run_tasks();

	static void reset()
	{
		_time = 0;
		for (size_t i = 0; i < _tasks.size(); ++i)
		{
			_tasks[i].timepoint = now();
		}
	}
protected:
	static __interrupt void on_interrupt();
};


class high_resolution_clock : public emb::monostate<high_resolution_clock>
{
private:
	static uint32_t _period;
	static const uint32_t sysclk_period_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
public:
	static void init(uint32_t period_us);
	static uint32_t counter() { return CPUTimer_getTimerCount(CPUTIMER1_BASE); }
	static uint64_t now() {	return static_cast<uint64_t>(_period - counter()) * sysclk_period_ns; }
	static void start() { CPUTimer_startTimer(CPUTIMER1_BASE); }
	static void stop() { CPUTimer_stopTimer(CPUTIMER1_BASE); }

	static void register_interrupt_handler(void (*handler)(void))
	{
		Interrupt_register(INT_TIMER1, handler);
		CPUTimer_enableInterrupt(CPUTIMER1_BASE);
		Interrupt_enable(INT_TIMER1);
	}
private:
	static __interrupt void on_interrupt();
};


class Timeout
{
private:
	const uint64_t _timeout;
	volatile uint64_t _start;
public:
	Timeout(uint64_t timeout = 0)
		: _timeout(timeout)
		, _start(system_clock::now())
	{}

	bool expired() volatile
	{
		if (_timeout == 0)
		{
			return false;
		}
		if ((system_clock::now() - _start) > _timeout)
		{
			return true;
		}
		return false;
	}

	void reset() volatile {	_start = system_clock::now(); }
};

} // namespace chrono

} // namespace mcu

