#ifdef STM32H7xx

#include "tests.h"


void mcu::tests::chrono_test() {
    auto taskLedToggle = [](int task_idx) {
        bsp::nucleo::led_green.toggle();
        return mcu::chrono::TaskStatus::success;
    };
    mcu::chrono::system_clock::add_task(taskLedToggle, std::chrono::milliseconds(100));
    mcu::chrono::system_clock::register_delayed_task([](){ bsp::nucleo::led_red.toggle(); }, std::chrono::milliseconds(200));

    for (auto i = 1; i <= 4; ++i) {
        mcu::delay(std::chrono::milliseconds(101));
        mcu::chrono::system_clock::run_tasks();
        if (i % 2 != 0) {
            EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::active);
        } else {
            EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::inactive);
        }

        if ((i < 2) || (i == 4)) {
            EMB_ASSERT_EQUAL(bsp::nucleo::led_red.read(), emb::gpio::State::inactive);
        } else {
            EMB_ASSERT_EQUAL(bsp::nucleo::led_red.read(), emb::gpio::State::active);
        }

        if (i == 2) {
            mcu::chrono::system_clock::register_delayed_task([](){ bsp::nucleo::led_red.toggle(); }, std::chrono::milliseconds(200));
        }
    }
}

#endif

