#ifdef STM32H7xx

#include "tests.h"


void mcu::tests::gpio_test()
{
    bsp::nucleo::led_green.reset();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::inactive);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 0);

    bsp::nucleo::led_green.set();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::active);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 1);

    bsp::nucleo::led_green.reset();
    bsp::nucleo::led_green.deinit();

    mcu::gpio::Config altConfig = bsp::nucleo::led_green_config;
    altConfig.active_state = emb::gpio::ActiveState::low;

    bsp::nucleo::led_green.init(altConfig);
    bsp::nucleo::led_green.reset();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::inactive);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 1);

    bsp::nucleo::led_green.set();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::active);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 0);

    bsp::nucleo::led_green.deinit();
    bsp::nucleo::led_green.init(bsp::nucleo::led_green_config);
}

#endif

