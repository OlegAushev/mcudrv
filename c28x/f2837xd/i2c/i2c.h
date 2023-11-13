#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include "../gpio/gpio.h"
#include <emblib/core.h>


namespace mcu {


namespace i2c {


SCOPED_ENUM_DECLARE_BEGIN(Peripheral) {
    i2ca,
    i2cb
} SCOPED_ENUM_DECLARE_END(Peripheral)


const size_t peripheral_count = 2;


SCOPED_ENUM_DECLARE_BEGIN(BitCount) {
    bc1 = I2C_BITCOUNT_1,
    bc2 = I2C_BITCOUNT_2,
    bc3 = I2C_BITCOUNT_3,
    bc4 = I2C_BITCOUNT_4,
    bc5 = I2C_BITCOUNT_5,
    bc6 = I2C_BITCOUNT_6,
    bc7 = I2C_BITCOUNT_7,
    bc8 = I2C_BITCOUNT_8
} SCOPED_ENUM_DECLARE_END(BitCount)


SCOPED_ENUM_DECLARE_BEGIN(DutyCycle) {
    dc33 = I2C_DUTYCYCLE_33,
    dc50 = I2C_DUTYCYCLE_50
} SCOPED_ENUM_DECLARE_END(DutyCycle)


struct Config {
    uint32_t bitrate;
    BitCount bitcount;
    DutyCycle duty_cycle;
    uint16_t slave_addr;
};


namespace impl {


struct Module {
    uint32_t base;
    Module(uint32_t base_) : base(base_) {}
};


extern const uint32_t i2c_bases[2];


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    impl::Module _module;
public:
    Module(Peripheral peripheral, const gpio::Config& sda_pin, const gpio::Config& scl_pin, const i2c::Config& config);
#ifdef CPU1
    static void transfer_control_to_cpu2(Peripheral peripheral, const gpio::Config& sdaPin, const gpio::Config& sclPin);
#endif
    Peripheral peripheral() const { return _peripheral; }
    uint32_t base() const { return _module.base; }
    void set_slave_address(uint16_t slave_addr) { I2C_setSlaveAddress(_module.base, slave_addr); }
    void enable() { I2C_enableModule(_module.base); }
    void disable() { I2C_disableModule(_module.base); }

    bool has_stop_condition() const { return I2C_getStopConditionStatus(_module.base); }
    bool is_bus_busy() const { return I2C_isBusBusy(_module.base); }
    uint16_t status() const { return I2C_getStatus(_module.base); }
    void clear_status(uint16_t status_flags) { I2C_clearStatus(_module.base, status_flags); }

    void set_mode(uint16_t mode) { I2C_setConfig(_module.base, mode); }
    void set_byte_count(uint16_t count) { I2C_setDataCount(_module.base, count); }
    void send_start() { I2C_sendStartCondition(_module.base); }
    void send_stop() { I2C_sendStopCondition(_module.base); }
    void send(uint8_t data) { I2C_putData(_module.base, data); }
    uint8_t recv() { return I2C_getData(_module.base); }

protected:
#ifdef CPU1
    static void _init_pins(const gpio::Config& sda_pin, const gpio::Config& scl_pin);
#endif
};


} // namespace i2c


} // namespace mcu


#endif
