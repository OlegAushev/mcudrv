#ifdef MCUDRV_STM32
#ifdef STM32F4xx

#include <mcudrv/stm32/f4/tests/tests.h>


void mcu::tests::chrono_test() {
#ifdef STM32F446_NUCLEO
    // OBSOLETE
    // auto taskLedToggle = [](size_t task_idx) {
    //     bsp::nucleo::led_green.toggle();
    //     return mcu::chrono::TaskStatus::success;
    // };
    // mcu::chrono::steady_clock::add_task(taskLedToggle, std::chrono::milliseconds(100));
    // mcu::chrono::steady_clock::register_delayed_task([](){ bsp::nucleo::led_red.toggle(); }, std::chrono::milliseconds(200));

    // for (auto i = 1; i <= 4; ++i) {
    //     mcu::delay(std::chrono::milliseconds(101));
    //     mcu::chrono::steady_clock::run_tasks();
    //     if (i % 2 != 0) {
    //         EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), mcu::gpio::pin_state::active);
    //     } else {
    //         EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), mcu::gpio::pin_state::inactive);
    //     }

    //     if ((i < 2) || (i == 4)) {
    //         EMB_ASSERT_EQUAL(bsp::nucleo::led_red.read(), mcu::gpio::pin_state::inactive);
    //     } else {
    //         EMB_ASSERT_EQUAL(bsp::nucleo::led_red.read(), mcu::gpio::pin_state::active);
    //     }

    //     if (i == 2) {
    //         mcu::chrono::steady_clock::register_delayed_task([](){ bsp::nucleo::led_red.toggle(); }, std::chrono::milliseconds(200));
    //     }
    // }
#endif
}


#endif
#endif
