#include <mcu_c28x/f2837xd/qep/mcu_qep.h>


namespace mcu {

namespace qep {

const uint32_t impl::qep_bases[3] = {EQEP1_BASE, EQEP2_BASE, EQEP3_BASE};
const uint32_t impl::qep_pie_int_nums[3] = {INT_EQEP1, INT_EQEP2, INT_EQEP3};


Module::Module(Peripheral peripheral, const gpio::Config& qepa_pin, const gpio::Config& qepb_pin,
		const gpio::Config& qepi_pin, const Config& config)
	: emb::c28x::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
	, _peripheral(peripheral)
	, _module(impl::qep_bases[peripheral.underlying_value()], config.int_flags, impl::qep_pie_int_nums[peripheral.underlying_value()])
{
#ifdef CPU1
	_init_pins(qepa_pin, qepb_pin, qepi_pin);
#endif

	// Configure the decoder
	EQEP_setDecoderConfig(_module.base,
			static_cast<uint16_t>(config.input_mode.underlying_value())
			| static_cast<uint16_t>(config.resolution.underlying_value())
			| static_cast<uint16_t>(config.swap_ab.underlying_value()));
	EQEP_setEmulationMode(_module.base, EQEP_EMULATIONMODE_RUNFREE);

	// Configure the position counter to reset on an index event
	EQEP_setPositionCounterConfig(_module.base,
			static_cast<EQEP_PositionResetMode>(config.reset_mode.underlying_value()), config.max_position);

	// Configure initial position
	EQEP_setPositionInitMode(_module.base, config.init_mode);
	EQEP_setInitialPosition(_module.base, config.init_position);

	// Enable the unit timer, setting the frequency
	EQEP_enableUnitTimer(_module.base, (mcu::sysclk_freq() / config.timeout_freq));

	// Configure the position counter to be latched on a unit time out
	EQEP_setLatchMode(_module.base, config.latch_mode);

	// Enable the eQEP1 module
	EQEP_enableModule(_module.base);

	// Configure and enable the edge-capture unit.
	EQEP_setCaptureConfig(_module.base, config.cap_prescaler, config.event_prescaler);
	EQEP_enableCapture(_module.base);
}


#ifdef CPU1
void Module::_init_pins(const gpio::Config& qepa_pin, const gpio::Config& qepb_pin,
		const gpio::Config& qepi_pin)
{
	GPIO_setPadConfig(qepa_pin.no, GPIO_PIN_TYPE_STD);
	GPIO_setPinConfig(qepa_pin.mux);

	if (qepb_pin.valid)
	{
		GPIO_setPadConfig(qepb_pin.no, GPIO_PIN_TYPE_STD);
		GPIO_setPinConfig(qepb_pin.mux);
	}

	if (qepi_pin.valid)
	{
		GPIO_setPadConfig(qepi_pin.no, GPIO_PIN_TYPE_STD);
		GPIO_setPinConfig(qepi_pin.mux);
	}
}
#endif

} // namespace qep

} // namespace mcu

