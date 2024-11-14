#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/tests/tests.h>
#include <emblib/scheduler.h>


void mcu::c28x::tests::gpio_test() {
#ifdef MCU_TESTS_ENABLED
#ifdef _LAUNCHXL_F28379D
    mcu::c28x::gpio::PinConfig out1cfg = {125, GPIO_125_GPIO125, mcu::c28x::gpio::Direction::output, emb::gpio::active_state::high, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};
    mcu::c28x::gpio::PinConfig out2cfg = {29, GPIO_29_GPIO29, mcu::c28x::gpio::Direction::output, emb::gpio::active_state::low, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};
    mcu::c28x::gpio::PinConfig in1cfg = {59, GPIO_59_GPIO59, mcu::c28x::gpio::Direction::input, emb::gpio::active_state::low, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};
    mcu::c28x::gpio::PinConfig in2cfg = {124, GPIO_124_GPIO124, mcu::c28x::gpio::Direction::input, emb::gpio::active_state::high, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};

    mcu::c28x::gpio::OutputPin out1(out1cfg);
    mcu::c28x::gpio::OutputPin out2(out2cfg);
    mcu::c28x::gpio::InputPin in1(in1cfg);
    mcu::c28x::gpio::InputPin in2(in2cfg);

    out1.reset();
    out2.reset();

    EMB_ASSERT_EQUAL(out1.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(in1.read(), emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(out2.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(in2.read(), emb::gpio::pin_state::active);

    EMB_ASSERT_EQUAL(out1.read_level(), 0);
    EMB_ASSERT_EQUAL(in1.read_level(), 0);
    EMB_ASSERT_EQUAL(out2.read_level(), 1);
    EMB_ASSERT_EQUAL(in2.read_level(), 1);

    out1.set();
    out2.set();

    EMB_ASSERT_EQUAL(out1.read(), emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(in1.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(out2.read(), emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(in2.read(), emb::gpio::pin_state::inactive);

    EMB_ASSERT_EQUAL(out1.read_level(), 1);
    EMB_ASSERT_EQUAL(in1.read_level(), 1);
    EMB_ASSERT_EQUAL(out2.read_level(), 0);
    EMB_ASSERT_EQUAL(in2.read_level(), 0);

    out1.toggle();
    out2.toggle();

    EMB_ASSERT_EQUAL(out1.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(in1.read(), emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(out2.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(in2.read(), emb::gpio::pin_state::active);

    EMB_ASSERT_EQUAL(out1.read_level(), 0);
    EMB_ASSERT_EQUAL(in1.read_level(), 0);
    EMB_ASSERT_EQUAL(out2.read_level(), 1);
    EMB_ASSERT_EQUAL(in2.read_level(), 1);

#elif defined(UNIT_TESTS_ENABLED)
#warning "LAUNCHXL is required for full testing."
#endif

#ifdef _LAUNCHXL_F28379D
    mcu::c28x::gpio::PinConfig out3Cfg = {27, GPIO_27_GPIO27, mcu::c28x::gpio::Direction::output, emb::gpio::active_state::high, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};
    mcu::c28x::gpio::PinConfig in3Cfg = {25, GPIO_25_GPIO25, mcu::c28x::gpio::Direction::input, emb::gpio::active_state::high, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};
    mcu::c28x::gpio::PinConfig in4Cfg = {25, GPIO_25_GPIO25, mcu::c28x::gpio::Direction::input, emb::gpio::active_state::low, mcu::c28x::gpio::Type::std, mcu::c28x::gpio::QualMode::sync, 1, mcu::c28x::gpio::MasterCore::cpu1};

    mcu::c28x::gpio::OutputPin out3(out3Cfg);
    mcu::c28x::gpio::InputPin in3(in3Cfg);
    mcu::c28x::gpio::InputDebouncer db1(in3, emb::chrono::milliseconds(10), emb::chrono::milliseconds(20), emb::chrono::milliseconds(30));

    EMB_ASSERT_EQUAL(in3.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(!db1.state_changed());

    out3.set(emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(in3.read(), emb::gpio::pin_state::active);
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(!db1.state_changed());
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(db1.state_changed());
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db1.state_changed());


    out3.set(emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(in3.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db1.state_changed());
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db1.state_changed());
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db1.state_changed());
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(db1.state_changed());
    db1.debounce();
    EMB_ASSERT_EQUAL(db1.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(!db1.state_changed());


    out3.set(emb::gpio::pin_state::inactive);
    mcu::c28x::gpio::InputPin in4(in4Cfg);
    mcu::c28x::gpio::InputDebouncer db2(in4, emb::chrono::milliseconds(10), emb::chrono::milliseconds(40), emb::chrono::milliseconds(20));
    EMB_ASSERT_EQUAL(in4.read(), emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(!db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    db2.debounce();
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db2.state_changed());

    out3.set(emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(in4.read(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(!db2.state_changed());

    out3.set(emb::gpio::pin_state::inactive);
    EMB_ASSERT_EQUAL(in4.read(), emb::gpio::pin_state::active);
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    EMB_ASSERT_TRUE(!db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    db2.debounce();
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::inactive);
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(db2.state_changed());
    db2.debounce();
    EMB_ASSERT_EQUAL(db2.state(), emb::gpio::pin_state::active);
    EMB_ASSERT_TRUE(!db2.state_changed());
#elif defined(UNIT_TESTS_ENABLED)
#warning "LAUNCHXL is required for full testing."
#endif
#endif
}


void TestingDelayedTask() {
    GPIO_writePin(34, 0);
}


void mcu::c28x::tests::chrono_test() {
#ifdef MCU_TESTS_ENABLED
    GPIO_writePin(34, 1);

    emb::scheduler::basic_scheduler::add_delayed_task(TestingDelayedTask, emb::chrono::milliseconds(200));
    DEVICE_DELAY_US(150000);
    emb::scheduler::basic_scheduler::run();
    EMB_ASSERT_EQUAL(GPIO_readPin(34), 1);
    DEVICE_DELAY_US(100000);
    emb::scheduler::basic_scheduler::run();
    EMB_ASSERT_EQUAL(GPIO_readPin(34), 0);

    GPIO_writePin(34, 1);

    emb::chrono::watchdog wd(emb::chrono::milliseconds(20));
    EMB_ASSERT_TRUE(!wd.bad());
    for (int i = 0; i < 15; ++i) {
        mcu::c28x::chrono::delay(emb::chrono::milliseconds(1));
        EMB_ASSERT_TRUE(!wd.bad());
    }
    mcu::c28x::chrono::delay(emb::chrono::milliseconds(6));
    EMB_ASSERT_TRUE(wd.bad());

    wd.reset();
    EMB_ASSERT_TRUE(wd.good());
    for (int i = 0; i < 15; ++i) {
        mcu::c28x::chrono::delay(emb::chrono::milliseconds(1));
        EMB_ASSERT_TRUE(wd.good());
    }
    mcu::c28x::chrono::delay(emb::chrono::milliseconds(6));
    EMB_ASSERT_TRUE(!wd.good());
#endif
}


void mcu::c28x::tests::crc_test() {
#ifdef MCU_TESTS_ENABLED
    uint16_t input1[10] = {0x0201, 0x0403, 0x0605, 0x0807, 0x0A09, 0x0C0B, 0x0E0D, 0x000F, 0x55AA, 0xAA55};
    uint32_t crc1 = mcu::c28x::crc::calc_crc32(input1, 20);
    EMB_ASSERT_EQUAL(crc1, 0x7805DACE);

    uint16_t input2[2] = {0x0201, 0x0003};
    uint32_t crc2 = mcu::c28x::crc::calc_crc32(input2, 3);
    EMB_ASSERT_EQUAL(crc2, 0x1B0D6951);

    uint16_t input3[1] = {0x0001};
    uint32_t crc3 = mcu::c28x::crc::calc_crc32(input3, 1);
    EMB_ASSERT_EQUAL(crc3, 0x4AC9A203);

    uint16_t input5[2] = {0x0201, 0x0403};
    uint8_t crc5 = mcu::c28x::crc::calc_crc8(input5, 4, CRC_parity_even);
    EMB_ASSERT_EQUAL(crc5, 0xE3);

    uint16_t input6[4] = {0x0100, 0x0302, 0x0504, 0x0706};
    uint8_t crc6 = mcu::c28x::crc::calc_crc8(input6, 7, CRC_parity_odd);
    EMB_ASSERT_EQUAL(crc6, 0xD8);

    uint16_t input7[4] = {0x0100, 0x0302, 0x0504, 0x0706};
    uint8_t crc7 = mcu::c28x::crc::calc_crc8(input7, 7, CRC_parity_even);
    EMB_ASSERT_EQUAL(crc7, 0x2F);

    struct {
        uint32_t crc : 8;
        uint32_t a : 8;
        uint32_t b : 8;
        uint32_t c : 8;
        uint32_t d : 8;
        uint32_t e : 8;
        uint32_t f : 8;
        uint32_t g : 8;
    } input8;
    input8.a = 0x10; input8.b = 0x20; input8.c = 0x30; input8.d = 0x40; input8.e = 0x50; input8.f = 0x60; input8.g = 0x70;
    uint8_t crc8 = mcu::c28x::crc::calc_crc8(reinterpret_cast<uint16_t*>(&input8), 7, CRC_parity_odd);
    EMB_ASSERT_EQUAL(crc8, 0xA3);

    struct {
        uint32_t a : 8;
        uint32_t b : 8;
        uint32_t c : 8;
        uint32_t d : 8;
        uint32_t e : 8;
        uint32_t f : 8;
        uint32_t g : 8;
        uint32_t crc : 8;
    } input9;
    input9.a = 0x10; input9.b = 0x20; input9.c = 0x30; input9.d = 0x40; input9.e = 0x50; input9.f = 0x60; input9.g = 0x70;
    uint8_t crc9 = mcu::c28x::crc::calc_crc8(reinterpret_cast<uint16_t*>(&input9), 7, CRC_parity_even);
    EMB_ASSERT_EQUAL(crc9, 0xA3);

    uint8_t input4[5] = {0x11, 0x22, 0x33, 0x44, 0x55};
    uint32_t crc4 = mcu::c28x::crc::calc_crc32_byte8(input4, 5);
    EMB_ASSERT_EQUAL(crc4, 0x4EFF913E);
#endif
}


#endif
