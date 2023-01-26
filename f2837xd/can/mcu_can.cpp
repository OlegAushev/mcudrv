#include <mcu_c28x/f2837xd/can/mcu_can.h>


namespace mcu {

namespace can {

const uint32_t impl::can_bases[2] = {CANA_BASE, CANB_BASE};
const uint32_t impl::can_pie_int_nums[2] = {INT_CANA0, INT_CANB0};

void (*Module::_on_interrupt_callbacks[peripheral_count])(Module*, uint32_t, uint16_t);


Module::Module(Peripheral peripheral, const gpio::Config& rx_pin, const gpio::Config& tx_pin, const Config& config)
	: emb::c28x::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
	, _peripheral(peripheral)
	, _module(impl::can_bases[peripheral.underlying_value()], impl::can_pie_int_nums[peripheral.underlying_value()])
{
#ifdef CPU1
	_init_pins(rx_pin, tx_pin);
#endif

	CAN_initModule(_module.base);
	CAN_selectClockSource(_module.base, CAN_CLOCK_SOURCE_SYS);

	switch (config.bitrate.native_value())
	{
		case Bitrate::bitrate_125k:
		case Bitrate::bitrate_500k:
			CAN_setBitRate(_module.base, mcu::sysclk_freq(), static_cast<uint32_t>(config.bitrate.underlying_value()), 16);
			break;
		case Bitrate::bitrate_1000k:
			CAN_setBitRate(_module.base, mcu::sysclk_freq(), static_cast<uint32_t>(config.bitrate.underlying_value()), 10);
			break;
	}

	CAN_setAutoBusOnTime(_module.base, 0);
	CAN_enableAutoBusOn(_module.base);

	if (config.mode != Mode::normal)
	{
		CAN_enableTestMode(_module.base, static_cast<uint16_t>(config.mode.underlying_value()));
	}

	CAN_startModule(_module.base);
}


#ifdef CPU1
void Module::transfer_control_to_cpu2(Peripheral peripheral, const gpio::Config& rx_pin, const gpio::Config& tx_pin)
{
	_init_pins(rx_pin, tx_pin);
	GPIO_setMasterCore(rx_pin.no, GPIO_CORE_CPU2);
	GPIO_setMasterCore(tx_pin.no, GPIO_CORE_CPU2);
	SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL8_CAN, peripheral.underlying_value()+1, SYSCTL_CPUSEL_CPU2);
}
#endif


void Module::register_interrupt_handler(void (*handler)(void))
{
	Interrupt_register(_module.pie_int_num, handler);
	CAN_enableInterrupt(_module.base, CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
	CAN_enableGlobalInterrupt(_module.base, CAN_GLOBAL_INT_CANINT0);
}


void Module::register_interrupt_callback(void (*callback)(Module*, uint32_t, uint16_t))
{
	switch (_peripheral.native_value())
	{
		case Peripheral::cana:
			register_interrupt_handler(on_interrupt<Peripheral::cana>);
			break;
		case Peripheral::canb:
			register_interrupt_handler(on_interrupt<Peripheral::canb>);
	}
	_on_interrupt_callbacks[_peripheral.underlying_value()] = callback;
}


#ifdef CPU1
void Module::_init_pins(const gpio::Config& rxPin, const gpio::Config& txPin)
{
	GPIO_setPinConfig(rxPin.mux);
	GPIO_setPinConfig(txPin.mux);
}
#endif

} // namespace can

} // namespace mcu

