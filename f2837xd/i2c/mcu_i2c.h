#pragma once


#include <emb_c28x/emb_core.h>
#include <mcu_c28x/f2837xd/gpio/mcu_gpio.h>
#include <mcu_c28x/f2837xd/system/mcu_system.h>
#include "driverlib.h"
#include "device.h"


namespace mcu {

namespace i2c {

SCOPED_ENUM_DECLARE_BEGIN(Peripheral)
{
	i2ca,
	i2cb
}
SCOPED_ENUM_DECLARE_END(Peripheral)


const size_t peripheral_count = 2;


SCOPED_ENUM_DECLARE_BEGIN(BitCount)
{
	bc1 = I2C_BITCOUNT_1,
	bc2 = I2C_BITCOUNT_2,
	bc3 = I2C_BITCOUNT_3,
	bc4 = I2C_BITCOUNT_4,
	bc5 = I2C_BITCOUNT_5,
	bc6 = I2C_BITCOUNT_6,
	bc7 = I2C_BITCOUNT_7,
	bc8 = I2C_BITCOUNT_8
}
SCOPED_ENUM_DECLARE_END(BitCount)


SCOPED_ENUM_DECLARE_BEGIN(DutyCycle)
{
	dc33 = I2C_DUTYCYCLE_33,
	dc50 = I2C_DUTYCYCLE_50
}
SCOPED_ENUM_DECLARE_END(DutyCycle)


struct Config
{
	uint32_t bitrate;
	BitCount bitcount;
	DutyCycle duty_cycle;
	uint16_t slave_addr;
};


namespace impl {

struct Module
{
	uint32_t base;
	Module(uint32_t base_) : base(base_) {}
};


extern const uint32_t i2c_bases[2];

} // namespace impl


class Module : public emb::c28x::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable
{
private:
	const Peripheral _peripheral;
	impl::Module _module;
public:
	Module(Peripheral peripheral, const gpio::Config& sda_pin, const gpio::Config& scl_pin, const i2c::Config& config);
#ifdef CPU1
	static void transfer_control_to_cpu2(Peripheral peripheral, const gpio::Config& sdaPin, const gpio::Config& sclPin);
#endif
	uint32_t base() const { return _module.base; }
	void set_slave_address(uint16_t slave_addr) { I2C_setSlaveAddress(_module.base, slave_addr); }
	void enable() { I2C_enableModule(_module.base); }
	void disable() { I2C_disableModule(_module.base); }
protected:
#ifdef CPU1
	static void _init_pins(const gpio::Config& sda_pin, const gpio::Config& scl_pin);
#endif
};

} // namespace i2c

} // namespace mcu

