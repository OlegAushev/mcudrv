#ifdef MCUDRV_STM32
#ifdef STM32F4xx


#include "tests.h"


void mcu::tests::gpio_test()
{
#ifdef STM32F446_NUCLEO
    bsp::nucleo::led_green.reset();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::inactive);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 0);

    bsp::nucleo::led_green.set();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::active);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 1);

    bsp::nucleo::led_green.reset();
    bsp::nucleo::led_green.deinit();

    mcu::gpio::Config alt_config = bsp::nucleo::led_green_config;
    alt_config.active_state = emb::gpio::ActiveState::low;

    bsp::nucleo::led_green.init(alt_config);
    bsp::nucleo::led_green.reset();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::inactive);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 1);

    bsp::nucleo::led_green.set();
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read(), emb::gpio::State::active);
    EMB_ASSERT_EQUAL(bsp::nucleo::led_green.read_level(), 0);

    bsp::nucleo::led_green.deinit();
    bsp::nucleo::led_green.init(bsp::nucleo::led_green_config);
#endif
}


#endif
#endif
