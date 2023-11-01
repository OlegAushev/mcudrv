#ifdef STM32F4xx

#include <mculib_stm32/f4/chrono/chrono.h>


/// @brief Mandatory HAL-lib tick handler.
extern "C" void SysTick_Handler()
{
    HAL_IncTick();
    mcu::chrono::system_clock::on_interrupt();
}


namespace mcu {

namespace chrono {

void system_clock::init() {
    set_initialized();
}


void system_clock::run_tasks() {
    for (size_t i = 0; i < _tasks.size(); ++i) {
        if (now() >= (_tasks[i].timepoint + _tasks[i].period)) {
            if (_tasks[i].func(i) == TaskStatus::success) {
                _tasks[i].timepoint = now();
            }
        }
    }


    if (_delayed_task_delay.count() != 0)
    {
        if (now() >= (_delayed_task_start + _delayed_task_delay))
        {
            _delayed_task();
            _delayed_task_delay = std::chrono::milliseconds(0);
        }
    }
}

} //namespace chrono

} // namespace mcu

#endif
