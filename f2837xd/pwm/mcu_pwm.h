#pragma once


#include <emb_c28x/emb_array.h>
#include <emb_c28x/emb_core.h>
#include "driverlib.h"
#include "device.h"

#include <math.h>
#include <mcu_c28x/f2837xd/gpio/mcu_gpio.h>


namespace mcu {

namespace pwm {

SCOPED_ENUM_DECLARE_BEGIN(State)
{
	off,
	on
}
SCOPED_ENUM_DECLARE_END(State)


SCOPED_ENUM_DECLARE_BEGIN(Peripheral)
{
	pwm1,
	pwm2,
	pwm3,
	pwm4,
	pwm5,
	pwm6,
	pwm7,
	pwm8,
	pwm9,
	pwm10,
	pwm11,
	pwm12
}
SCOPED_ENUM_DECLARE_END(Peripheral)


SCOPED_ENUM_DECLARE_BEGIN(PhaseCount)
{
	one = 1,
	three = 3,
	six = 6
}
SCOPED_ENUM_DECLARE_END(PhaseCount)


SCOPED_ENUM_DECLARE_BEGIN(ClockDivider)
{
	divider1 = EPWM_CLOCK_DIVIDER_1,
	divider2 = EPWM_CLOCK_DIVIDER_2,
	divider4 = EPWM_CLOCK_DIVIDER_4,
	divider8 = EPWM_CLOCK_DIVIDER_8,
	divider16 = EPWM_CLOCK_DIVIDER_16,
	divider32 = EPWM_CLOCK_DIVIDER_32,
	divider64 = EPWM_CLOCK_DIVIDER_64,
	divider128 = EPWM_CLOCK_DIVIDER_128
}
SCOPED_ENUM_DECLARE_END(ClockDivider)


SCOPED_ENUM_DECLARE_BEGIN(HsClockDivider)
{
	divider1 = EPWM_HSCLOCK_DIVIDER_1,
	divider2 = EPWM_HSCLOCK_DIVIDER_2,
	divider4 = EPWM_HSCLOCK_DIVIDER_4,
	divider6 = EPWM_HSCLOCK_DIVIDER_6,
	divider8 = EPWM_HSCLOCK_DIVIDER_8,
	divider10 = EPWM_HSCLOCK_DIVIDER_10,
	divider12 = EPWM_HSCLOCK_DIVIDER_12,
	divider14 = EPWM_HSCLOCK_DIVIDER_14
}
SCOPED_ENUM_DECLARE_END(HsClockDivider)


SCOPED_ENUM_DECLARE_BEGIN(OperatingMode)
{
	active_high_complementary,
	active_low_complementary,
	pass_through
}
SCOPED_ENUM_DECLARE_END(OperatingMode)


SCOPED_ENUM_DECLARE_BEGIN(CounterMode)
{
	up = EPWM_COUNTER_MODE_UP,
	down = EPWM_COUNTER_MODE_DOWN,
	updown = EPWM_COUNTER_MODE_UP_DOWN
}
SCOPED_ENUM_DECLARE_END(CounterMode)


SCOPED_ENUM_DECLARE_BEGIN(OutputSwap)
{
	no,
	yes
}
SCOPED_ENUM_DECLARE_END(OutputSwap)


SCOPED_ENUM_DECLARE_BEGIN(CounterCompareModule)
{
	a = EPWM_COUNTER_COMPARE_A,
	b = EPWM_COUNTER_COMPARE_B
}
SCOPED_ENUM_DECLARE_END(CounterCompareModule)


template <PhaseCount::enum_type PhaseCount>
struct Config
{
	Peripheral peripheral[PhaseCount];
	float switching_freq;
	float deadtime_ns;
	uint32_t clock_prescaler;	// must be the product of clkDivider and hsclkDivider
	ClockDivider clk_divider;
	HsClockDivider hsclk_divider;
	OperatingMode operating_mode;
	CounterMode counter_mode;
	OutputSwap output_swap;
	uint16_t event_interrupt_source;
	bool enable_adc_trigger[2];
	EPWM_ADCStartOfConversionSource adc_trigger_source[2];
};


namespace impl {

template <PhaseCount::enum_type Phases>
struct Module
{
	uint32_t base[Phases];
	uint32_t pie_event_int_num;
	uint32_t pie_trip_int_num;
};


extern const uint32_t pwm_bases[12];
extern const uint32_t pwm_pie_event_int_nums[12];
extern const uint32_t pwm_pie_trip_int_nums[12];
extern const uint32_t pwm_pin_outa_configs[12];
extern const uint32_t pwm_pin_outb_configs[12];

} // namespace impl


template <PhaseCount::enum_type Phases>
class Module : private emb::noncopyable
{
private:
	// there is a divider ( EPWMCLKDIV ) of the system clock
	// which defaults to EPWMCLK = SYSCLKOUT/2, fclk(epwm)max = 100 MHz
	static const uint32_t pwm_clk_freq = DEVICE_SYSCLK_FREQ / 2;
	static const uint32_t pwm_clk_cycle_ns = 1000000000 / pwm_clk_freq;
	const uint32_t _timebase_clk_freq;
	const uint32_t _timebase_cycle_ns;

	Peripheral _peripheral[Phases];
	impl::Module<Phases> _module;
	CounterMode _counter_mode;
	float _switching_freq;
	uint16_t _deadtime_cycles;

	uint16_t _period;		// TBPRD register value
	uint16_t _phase_shift[Phases];	// TBPHS registers values

	State _state;
public:
	Module(const pwm::Config<Phases>& config)
		: _timebase_clk_freq(pwm_clk_freq / config.clock_prescaler)
		, _timebase_cycle_ns(pwm_clk_cycle_ns * config.clock_prescaler)
		, _counter_mode(config.counter_mode)
		, _switching_freq(config.switching_freq)
		, _deadtime_cycles(config.deadtime_ns / _timebase_cycle_ns)
		, _state(State::off)
	{
		for (size_t i = 0; i < Phases; ++i)
		{
			_peripheral[i] = config.peripheral[i];
			_module.base[i] = impl::pwm_bases[config.peripheral[i].underlying_value()];
		}
		_module.pie_event_int_num = impl::pwm_pie_event_int_nums[config.peripheral[0].underlying_value()];
		_module.pie_trip_int_num = impl::pwm_pie_trip_int_nums[config.peripheral[0].underlying_value()];

		for (size_t i = 0; i < Phases; ++i)
		{
			_phase_shift[i] = 0;
		}

#ifdef CPU1
		_init_pins(config);
#else
		EMB_UNUSED(impl::pwm_pin_outa_configs);
		EMB_UNUSED(impl::pwm_pin_outb_configs);
#endif

		// Disable sync, freeze clock to PWM
		SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

		/* ========================================================================== */
		// Calculate TBPRD value
		switch (config.counter_mode.native_value())
		{
			case CounterMode::up:
			case CounterMode::down:
				_period = (_timebase_clk_freq / _switching_freq) - 1;
				break;
			case CounterMode::updown:
				_period = (_timebase_clk_freq / _switching_freq) / 2;
				break;
		}

		for (size_t i = 0; i < Phases; ++i)
		{
			EPWM_setTimeBasePeriod(_module.base[i], _period);
			EPWM_setTimeBaseCounter(_module.base[i], 0);

			/* ========================================================================== */
			// Clock prescaler
			EPWM_setClockPrescaler(_module.base[i],
					static_cast<EPWM_ClockDivider>(config.clk_divider.underlying_value()),
					static_cast<EPWM_HSClockDivider>(config.hsclk_divider.underlying_value()));

			/* ========================================================================== */
			// Compare values
			EPWM_setCounterCompareValue(_module.base[i], EPWM_COUNTER_COMPARE_A, 0);

			/* ========================================================================== */
			// Counter mode
			EPWM_setTimeBaseCounterMode(_module.base[i],
					static_cast<EPWM_TimeBaseCountMode>(config.counter_mode.underlying_value()));

#ifdef CPU1
			/* ========================================================================== */
			// Sync input source for the EPWM signals
			switch (_module.base[i])
			{
				case EPWM4_BASE:
					SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM4, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
					break;
				case EPWM7_BASE:
					SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM7, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
					break;
				case EPWM10_BASE:
					SysCtl_setSyncInputConfig(SYSCTL_SYNC_IN_EPWM10, SYSCTL_SYNC_IN_SRC_EPWM1SYNCOUT);
					break;
				default:
					break;
			}
#endif

			/* ========================================================================== */
			// Sync out pulse event
			switch (Phases)
			{
				case PhaseCount::six:
				case PhaseCount::three:
					if ((i == 0) && (_module.base[i] == EPWM1_BASE))
					{
						// EPWM1 is master
						EPWM_setSyncOutPulseMode(_module.base[i], EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
					}
					else
					{
						// other modules sync is pass-through
						EPWM_setSyncOutPulseMode(_module.base[i], EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
					}
					break;
				case PhaseCount::one:
					if (_module.base[i] == EPWM1_BASE)
					{
						EPWM_setSyncOutPulseMode(_module.base[i], EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
					}
					else
					{
						EPWM_setSyncOutPulseMode(_module.base[i], EPWM_SYNC_OUT_PULSE_ON_EPWMxSYNCIN);
					}
					break;
			}

			/* ========================================================================== */
			// Time-base counter synchronization and phase shift
			switch (Phases)
			{
				case PhaseCount::six:
				case PhaseCount::three:
					if ((i == 0) && (_module.base[i] == EPWM1_BASE))
					{
						// EPWM1 is master, EPWM4,7,10 are synced to it
						// master has no phase shift
						EPWM_disablePhaseShiftLoad(_module.base[i]);
						EPWM_setPhaseShift(_module.base[i], 0);
					}
					else
					{
						EPWM_enablePhaseShiftLoad(_module.base[i]);
						switch (config.counter_mode.native_value())
						{
							case CounterMode::up:
								EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_UP_AFTER_SYNC);
								// 2 x EPWMCLK - delay from internal master module to slave modules, p.1876
								EPWM_setPhaseShift(_module.base[i], 2 + _phase_shift[i]);
								break;
							case CounterMode::down:
								EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
								EPWM_setPhaseShift(_module.base[i], _phase_shift[i]);
								break;
							case CounterMode::updown:
								EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_UP_AFTER_SYNC);
								// 2 x EPWMCLK - delay from internal master module to slave modules, p.1876
								EPWM_setPhaseShift(_module.base[i], 2 + _phase_shift[i]);
								break;
						}
					}
					break;
				case PhaseCount::one:
					if (_module.base[i] == EPWM1_BASE)
					{
						EPWM_disablePhaseShiftLoad(_module.base[i]);
						EPWM_setPhaseShift(_module.base[i], 0);
					}
					else
					{
						EPWM_enablePhaseShiftLoad(_module.base[i]);
						switch (config.counter_mode.native_value())
						{
							case CounterMode::up:
								EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_UP_AFTER_SYNC);
								// 2 x EPWMCLK - delay from internal master module to slave modules, p.1876
								EPWM_setPhaseShift(_module.base[i], 2 + _phase_shift[i]);
								break;
							case CounterMode::down:
								EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_DOWN_AFTER_SYNC);
								EPWM_setPhaseShift(_module.base[i], _phase_shift[i]);
								break;
							case CounterMode::updown:
								EPWM_setCountModeAfterSync(_module.base[i], EPWM_COUNT_MODE_UP_AFTER_SYNC);
								// 2 x EPWMCLK - delay from internal master module to slave modules, p.1876
								EPWM_setPhaseShift(_module.base[i], 2 + _phase_shift[i]);
								break;
						}
					}
					break;
			}

			/* ========================================================================== */
			// Shadowing
			EPWM_selectPeriodLoadEvent(_module.base[i], EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
			EPWM_setCounterCompareShadowLoadMode(_module.base[i], EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO);
			EPWM_setCounterCompareShadowLoadMode(_module.base[i], EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO);
			EPWM_setActionQualifierContSWForceShadowMode(_module.base[i], EPWM_AQ_SW_IMMEDIATE_LOAD);

			/* ========================================================================== */
			// PWMxA(CMPA) actions
				// PWMxA configuration for typical waveforms
				// Typically only PWMxA is used by dead-band submodule
			switch (config.counter_mode.native_value())
			{
				case CounterMode::up:
					EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_A,
							EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
							EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
					break;
				case CounterMode::down:
					EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_A,
							EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
							EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
					break;
				case CounterMode::updown:
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
							EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_A,
							EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
					break;
			}

			// PWMxB(CMPB) actions
			switch (config.counter_mode.native_value())
			{
				case CounterMode::up:
					EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_B,
							EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
							EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
					break;
				case CounterMode::down:
					EPWM_setActionQualifierAction(_module.base[i],	EPWM_AQ_OUTPUT_B,
							EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
							EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
					break;
				case CounterMode::updown:
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
							EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
					EPWM_setActionQualifierAction(_module.base[i], EPWM_AQ_OUTPUT_B,
							EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
					break;
			}

			/* ========================================================================== */
			// Dead-Band
			EPWM_setDeadBandControlShadowLoadMode(_module.base[i], EPWM_DB_LOAD_ON_CNTR_ZERO);

			switch (config.operating_mode.native_value())
			{
				case OperatingMode::active_high_complementary:
					EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_FED, true);
					EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_RED, true);
					EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_HIGH);
					EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
					break;
				case OperatingMode::active_low_complementary:
					EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_FED, true);
					EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_RED, true);
					EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_RED, EPWM_DB_POLARITY_ACTIVE_LOW);
					EPWM_setDeadBandDelayPolarity(_module.base[i], EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_HIGH);
					break;
				case OperatingMode::pass_through:
					EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_FED, false);
					EPWM_setDeadBandDelayMode(_module.base[i], EPWM_DB_RED, false);
					break;
			}

			EPWM_setRisingEdgeDeadBandDelayInput(_module.base[i], EPWM_DB_INPUT_EPWMA);
			EPWM_setFallingEdgeDeadBandDelayInput(_module.base[i], EPWM_DB_INPUT_EPWMA);
			EPWM_setRisingEdgeDelayCount(_module.base[i], _deadtime_cycles);
			EPWM_setFallingEdgeDelayCount(_module.base[i], _deadtime_cycles);
			EPWM_setDeadBandCounterClock(_module.base[i], EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

			switch (config.output_swap.native_value())
			{
				case OutputSwap::no:
					EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_A, false);
					EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_B, false);
					break;
				case OutputSwap::yes:
					EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_A, true);
					EPWM_setDeadBandOutputSwapMode(_module.base[i], EPWM_DB_OUTPUT_B, true);
					break;
			}

			/* ========================================================================== */
			// Trip-Zone actions
			switch (config.operating_mode.native_value())
			{
				case OperatingMode::active_high_complementary:
					EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
					EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
					break;
				case OperatingMode::active_low_complementary:
					EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_HIGH);
					EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_HIGH);
					break;
				case OperatingMode::pass_through:
					EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZA, EPWM_TZ_ACTION_LOW);
					EPWM_setTripZoneAction(_module.base[i], EPWM_TZ_ACTION_EVENT_TZB, EPWM_TZ_ACTION_LOW);
					break;
			}

			EPWM_clearOneShotTripZoneFlag(_module.base[i], EPWM_TZ_OST_FLAG_OST1);
			EPWM_clearTripZoneFlag(_module.base[i], EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
		}

		/* ========================================================================== */
		// ADC Trigger configuration, only first module triggers ADC
		if (config.enable_adc_trigger[0])
		{
			EPWM_setADCTriggerSource(_module.base[0], EPWM_SOC_A, config.adc_trigger_source[0]);
			EPWM_setADCTriggerEventPrescale(_module.base[0], EPWM_SOC_A, 1);
			EPWM_enableADCTrigger(_module.base[0], EPWM_SOC_A);
		}

		if (config.enable_adc_trigger[1])
		{
			EPWM_setADCTriggerSource(_module.base[0], EPWM_SOC_B, config.adc_trigger_source[1]);
			EPWM_setADCTriggerEventPrescale(_module.base[0], EPWM_SOC_B, 1);
			EPWM_enableADCTrigger(_module.base[0], EPWM_SOC_B);
		}

		/* ========================================================================== */
		// Interrupts, only interrupt on first module is required
		EPWM_setInterruptSource(_module.base[0], config.event_interrupt_source);
		EPWM_setInterruptEventCount(_module.base[0], 1U);

		_init_custom_options();

		stop();

		// Enable sync and clock to PWM
		SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
	}

#ifdef CPU1
	void init_tripzone(const gpio::Input& pin, XBAR_InputNum xbar_input)
	{
		assert(static_cast<uint32_t>(xbar_input) <= static_cast<uint32_t>(XBAR_INPUT3));

		switch (pin.config().active_state.native_value())
		{
			case emb::gpio::ActiveState::low:
				GPIO_setPadConfig(pin.config().no, GPIO_PIN_TYPE_PULLUP);
				break;
			case emb::gpio::ActiveState::high:
				GPIO_setPadConfig(pin.config().no, GPIO_PIN_TYPE_INVERT);
				break;
		}

		GPIO_setPinConfig(pin.config().mux);
		GPIO_setDirectionMode(pin.config().no, GPIO_DIR_MODE_IN);
		GPIO_setQualificationMode(pin.config().no, GPIO_QUAL_ASYNC);

		XBAR_setInputPin(xbar_input, pin.config().no);
		uint16_t tripzone_signal;
		switch (xbar_input)
		{
			case XBAR_INPUT1:
				tripzone_signal = EPWM_TZ_SIGNAL_OSHT1;
				break;
			case XBAR_INPUT2:
				tripzone_signal = EPWM_TZ_SIGNAL_OSHT2;
				break;

			case XBAR_INPUT3:
				tripzone_signal = EPWM_TZ_SIGNAL_OSHT3;
				break;
			default:
				tripzone_signal = EPWM_TZ_SIGNAL_OSHT3;
				break;
		}

		for (size_t i = 0; i < Phases; ++i)
		{
			// Enable tzSignal as one shot trip source
			EPWM_enableTripZoneSignals(_module.base[i], tripzone_signal);
		}
	}
#endif

#ifdef CPU1
	void transfer_control_to_cpu2()
	{
		SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);	// Disable sync(Freeze clock to PWM as well)

		for (size_t i = 0; i < PhaseCount; ++i)
		{
			GPIO_setMasterCore(_module.instance[i] * 2, GPIO_CORE_CPU2);
			GPIO_setMasterCore(_module.instance[i] * 2 + 1, GPIO_CORE_CPU2);
		}

		for (size_t i = 0; i < PhaseCount; ++i)
		{
			SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL0_EPWM,
					static_cast<uint16_t>(_module.instance[i])+1, SYSCTL_CPUSEL_CPU2);
		}

		SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);	// Enable sync and clock to PWM
	}
#endif

	uint32_t base() const { return _module.base[0]; }
	uint16_t period() const { return _period; }
	float freq() const { return _switching_freq; }

	void set_freq(float freq)
	{
		_switching_freq = freq;
		switch (_counter_mode)
		{
			case CounterMode::up:
			case CounterMode::down:
				_period = (_timebase_clk_freq / _switching_freq) - 1;
				break;
			case CounterMode::updown:
				_period = (_timebase_clk_freq / _switching_freq) / 2;
				break;
		}

		for (size_t i = 0; i < PhaseCount; ++i)
		{
			EPWM_setTimeBasePeriod(_module.base[i], _period);
		}
	}

	void set_compare_values(const uint16_t cmp_values[], CounterCompareModule cmp_module = CounterCompareModule::a)
	{
		for (size_t i = 0; i < PhaseCount; ++i)
		{
			EPWM_setCounterCompareValue(_module.base[i],
							static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
							cmp_values[i]);
		}
	}

	void set_compare_values(const emb::Array<uint16_t, Phases>& cmp_values, CounterCompareModule cmp_module = CounterCompareModule::a)
	{
		for (size_t i = 0; i < PhaseCount; ++i)
		{
			EPWM_setCounterCompareValue(_module.base[i],
							static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
							cmp_values[i]);
		}
	}

	void set_compare_value(uint16_t cmp_value, CounterCompareModule cmp_module = CounterCompareModule::a)
	{
		for (size_t i = 0; i < PhaseCount; ++i)
		{
			EPWM_setCounterCompareValue(_module.base[i],
							static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
							cmp_value);
		}
	}

	void set_duty_cycles(const emb::Array<float, Phases>& duty_cycles, CounterCompareModule cmp_module = CounterCompareModule::a)
	{
		for (size_t i = 0; i < Phases; ++i)
		{
			EPWM_setCounterCompareValue(_module.base[i],
							static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
							static_cast<uint16_t>(duty_cycles[i] * _period));
		}
	}

	void set_duty_cycle(float duty_cycle, CounterCompareModule cmp_module = CounterCompareModule::a)
	{
		for (size_t i = 0; i < PhaseCount; ++i)
		{
			EPWM_setCounterCompareValue(_module.base[i],
							static_cast<EPWM_CounterCompareModule>(cmp_module.underlying_value()),
							static_cast<uint16_t>(duty_cycle * _period));
		}
	}

	void start()
	{
		_state = State::on;
		for (size_t i = 0; i < Phases; ++i)
		{
			EPWM_clearTripZoneFlag(_module.base[i], EPWM_TZ_INTERRUPT | EPWM_TZ_FLAG_OST);
		}
	}

	void stop()
	{
		for (size_t i = 0; i < Phases; ++i)
		{
			EPWM_forceTripZoneEvent(_module.base[i], EPWM_TZ_FORCE_EVENT_OST);
		}
		_state = State::off;
	}

	State state() const { return _state; }
	void enable_event_interrupts() { EPWM_enableInterrupt(_module.base[0]); }
	void enable_trip_interrupts() { EPWM_enableTripZoneInterrupt(_module.base[0], EPWM_TZ_INTERRUPT_OST); }
	void disable_event_interrupts() { EPWM_disableInterrupt(_module.base[0]); }
	void disable_trip_interrupts() { EPWM_disableTripZoneInterrupt(_module.base[0], EPWM_TZ_INTERRUPT_OST); }

	void register_event_interrupt_handler(void (*handler)(void))
	{
		Interrupt_register(_module.pie_event_int_num, handler);
		Interrupt_enable(_module.pie_event_int_num);
	}

	void register_trip_interrupt_handler(void (*handler)(void))
	{
		Interrupt_register(_module.pie_trip_int_num, handler);
		Interrupt_enable(_module.pie_trip_int_num);
	}

	void acknowledge_event_interrupt()
	{
		EPWM_clearEventTriggerInterruptFlag(_module.base[0]);
		Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
	}

	void acknowledge_trip_interrupt() { Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2); }
protected:
#ifdef CPU1
	void _init_pins(const pwm::Config<Phases>& config)
	{
		for (size_t i = 0; i < Phases; ++i)
		{
			GPIO_setPadConfig(config.peripheral[i].underlying_value() * 2, GPIO_PIN_TYPE_STD);
			GPIO_setPadConfig(config.peripheral[i].underlying_value() * 2 + 1, GPIO_PIN_TYPE_STD);
			GPIO_setPinConfig(impl::pwm_pin_outa_configs[config.peripheral[i].underlying_value()]);
			GPIO_setPinConfig(impl::pwm_pin_outb_configs[config.peripheral[i].underlying_value()]);
		}
	}
#endif
	void _init_custom_options();
};

} // namespace pwm

} // namespace mcu

