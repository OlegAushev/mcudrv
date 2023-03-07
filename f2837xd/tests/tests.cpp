#include <mculib_c28x/f2837xd/tests/tests.h>


void mcu::tests::gpio()
{
#ifdef _LAUNCHXL_F28379D
	mcu::gpio::Config led_blue_cfg(31, GPIO_31_GPIO31, mcu::gpio::Direction::output, emb::gpio::ActiveState::high, mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1);
	mcu::gpio::Config led_red_cfg(34, GPIO_34_GPIO34, mcu::gpio::Direction::output, emb::gpio::ActiveState::low, mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1);

	mcu::gpio::Output led_blue(led_blue_cfg);
	mcu::gpio::Output led_red(led_red_cfg);

	GPIO_writePin(31, 1);
	GPIO_writePin(34, 1);

	EMB_ASSERT_EQUAL(led_blue.read(), emb::gpio::State::active);
	EMB_ASSERT_EQUAL(led_red.read(), emb::gpio::State::inactive);

	led_blue.set(emb::gpio::State::inactive);	// led - on
	led_red.set(emb::gpio::State::inactive);		// led - off
	DEVICE_DELAY_US(100000);

	EMB_ASSERT_EQUAL(GPIO_readPin(31), 0);
	EMB_ASSERT_EQUAL(GPIO_readPin(34), 1);
	EMB_ASSERT_EQUAL(led_blue.read(), emb::gpio::State::inactive);
	EMB_ASSERT_EQUAL(led_red.read(), emb::gpio::State::inactive);

	led_blue.set(emb::gpio::State::active);		// led - off
	led_red.set(emb::gpio::State::active);		// led - on
	DEVICE_DELAY_US(100000);

	EMB_ASSERT_EQUAL(GPIO_readPin(31), 1);
	EMB_ASSERT_EQUAL(GPIO_readPin(34), 0);
	EMB_ASSERT_EQUAL(led_blue.read(), emb::gpio::State::active);
	EMB_ASSERT_EQUAL(led_red.read(), emb::gpio::State::active);

	GPIO_writePin(31, 1);
	GPIO_writePin(34, 1);
#elif defined(ON_TARGET_TEST)
#warning "LAUNCHXL is required for full testing."
#endif

#ifdef _LAUNCHXL_F28379D
	mcu::gpio::Config outCfg(27, GPIO_27_GPIO27, mcu::gpio::Direction::output, emb::gpio::ActiveState::high, mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1);
	mcu::gpio::Config in1Cfg(25, GPIO_25_GPIO25, mcu::gpio::Direction::input, emb::gpio::ActiveState::high, mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1);
	mcu::gpio::Config in2Cfg(25, GPIO_25_GPIO25, mcu::gpio::Direction::input, emb::gpio::ActiveState::low, mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1);

	mcu::gpio::Output out(outCfg);
	mcu::gpio::Input in1(in1Cfg);
	mcu::gpio::InputDebouncer db1(in1, emb::chrono::milliseconds(10), emb::chrono::milliseconds(20), emb::chrono::milliseconds(30));

	EMB_ASSERT_EQUAL(in1.read(), emb::gpio::State::inactive);
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(!db1.state_changed());

	out.set(emb::gpio::State::active);
	EMB_ASSERT_EQUAL(in1.read(), emb::gpio::State::active);
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(!db1.state_changed());
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(db1.state_changed());
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db1.state_changed());


	out.set(emb::gpio::State::inactive);
	EMB_ASSERT_EQUAL(in1.read(), emb::gpio::State::inactive);
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db1.state_changed());
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db1.state_changed());
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db1.state_changed());
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(db1.state_changed());
	db1.debounce();
	EMB_ASSERT_EQUAL(db1.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(!db1.state_changed());


	out.set(emb::gpio::State::inactive);
	mcu::gpio::Input in2(in2Cfg);
	mcu::gpio::InputDebouncer db2(in2, emb::chrono::milliseconds(10), emb::chrono::milliseconds(40), emb::chrono::milliseconds(20));
	EMB_ASSERT_EQUAL(in2.read(), emb::gpio::State::active);
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(!db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	db2.debounce();
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db2.state_changed());

	out.set(emb::gpio::State::active);
	EMB_ASSERT_EQUAL(in2.read(), emb::gpio::State::inactive);
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(!db2.state_changed());

	out.set(emb::gpio::State::inactive);
	EMB_ASSERT_EQUAL(in2.read(), emb::gpio::State::active);
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	EMB_ASSERT_TRUE(!db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	db2.debounce();
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::inactive);
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(db2.state_changed());
	db2.debounce();
	EMB_ASSERT_EQUAL(db2.state(), emb::gpio::State::active);
	EMB_ASSERT_TRUE(!db2.state_changed());
#elif defined(TEST_BUILD)
#warning "LAUNCHXL is required for full testing."
#endif
}


void TestingDelayedTask()
{
	GPIO_writePin(34, 0);
}


void mcu::tests::chrono()
{
	GPIO_writePin(34, 1);

	mcu::chrono::system_clock::register_delayed_task(TestingDelayedTask, emb::chrono::milliseconds(200));
	DEVICE_DELAY_US(150000);
	mcu::chrono::system_clock::run_tasks();
	EMB_ASSERT_EQUAL(GPIO_readPin(34), 1);
	DEVICE_DELAY_US(100000);
	mcu::chrono::system_clock::run_tasks();
	EMB_ASSERT_EQUAL(GPIO_readPin(34), 0);

	GPIO_writePin(34, 1);

	mcu::chrono::Timeout timeout(emb::chrono::milliseconds(20));
	EMB_ASSERT_TRUE(!timeout.expired());
	for (size_t i = 0; i < 15; ++i)
	{
		mcu::delay_us(1000);
		EMB_ASSERT_TRUE(!timeout.expired());
	}
	mcu::delay_us(6000);
	EMB_ASSERT_TRUE(timeout.expired());

	timeout.reset();
	EMB_ASSERT_TRUE(!timeout.expired());
	for (size_t i = 0; i < 15; ++i)
	{
		mcu::delay_us(1000);
		EMB_ASSERT_TRUE(!timeout.expired());
	}
	mcu::delay_us(6000);
	EMB_ASSERT_TRUE(timeout.expired());
}


void mcu::tests::crc()
{
	uint16_t input1[10] = {0x0201, 0x0403, 0x0605, 0x0807, 0x0A09, 0x0C0B, 0x0E0D, 0x000F, 0x55AA, 0xAA55};
	uint32_t crc1 = mcu::crc::calc_crc32(input1, 20);
	EMB_ASSERT_EQUAL(crc1, 0x7805DACE);

	uint16_t input2[2] = {0x0201, 0x0003};
	uint32_t crc2 = mcu::crc::calc_crc32(input2, 3);
	EMB_ASSERT_EQUAL(crc2, 0x1B0D6951);

	uint16_t input3[1] = {0x0001};
	uint32_t crc3 = mcu::crc::calc_crc32(input3, 1);
	EMB_ASSERT_EQUAL(crc3, 0x4AC9A203);

	uint8_t input4[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
	uint32_t crc4 = mcu::crc::calc_crc32_byte8(input4, 5);
	EMB_ASSERT_EQUAL(crc4, 0x4EFF913E);
}




