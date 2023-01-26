#pragma once


#include "driverlib.h"
#include "device.h"
#include <assert.h>
#include <emb_c28x/emb_interfaces/emb_gpio.h>
#include <mcu_c28x/f2837xd/system/mcu_system.h>


namespace mcu {

namespace gpio {

SCOPED_ENUM_DECLARE_BEGIN(Type)
{
	std = GPIO_PIN_TYPE_STD,
	pullup = GPIO_PIN_TYPE_PULLUP,
	invert = GPIO_PIN_TYPE_INVERT,
	open_drain = GPIO_PIN_TYPE_OD
}
SCOPED_ENUM_DECLARE_END(Type)


SCOPED_ENUM_DECLARE_BEGIN(Direction)
{
	input = GPIO_DIR_MODE_IN,
	output = GPIO_DIR_MODE_OUT
}
SCOPED_ENUM_DECLARE_END(Direction)


SCOPED_ENUM_DECLARE_BEGIN(QualMode)
{
	sync = GPIO_QUAL_SYNC,
	sample3 = GPIO_QUAL_3SAMPLE,
	sample6 = GPIO_QUAL_6SAMPLE,
	async = GPIO_QUAL_ASYNC
}
SCOPED_ENUM_DECLARE_END(QualMode)


SCOPED_ENUM_DECLARE_BEGIN(MasterCore)
{
	cpu1 = GPIO_CORE_CPU1,
	cpu1_cla1 = GPIO_CORE_CPU1_CLA1,
	cpu2 = GPIO_CORE_CPU2,
	cpu2_cla1 = GPIO_CORE_CPU2_CLA1
}
SCOPED_ENUM_DECLARE_END(MasterCore)


namespace impl {

extern const uint32_t pie_xint_nums[5];
extern const uint16_t pie_xint_groups[5];

} // namespace impl


struct Config
{
	bool valid;
	uint32_t no;
	uint32_t mux;
	Direction direction;
	emb::gpio::ActiveState active_state;
	Type type;
	QualMode qual_mode;
	uint32_t qual_period;
	MasterCore master_core;

	Config() : valid(false) {}

	Config(uint32_t no_, uint32_t mux_, Direction direction_, emb::gpio::ActiveState active_state_,
			Type type_, QualMode qual_mode_, uint32_t qual_period_,
			MasterCore master_core_ = MasterCore::cpu1)
		: valid(true)
		, no(no_)
		, mux(mux_)
		, direction(direction_)
		, active_state(active_state_)
		, type(type_)
		, qual_mode(qual_mode_)
		, qual_period(qual_period_)
		, master_core(master_core_)
	{}

	Config(uint32_t no_, uint32_t mux_)
		: valid(false)
		, no(no_)
		, mux(mux_)
	{}

	Config(traits::unused) : valid(false) {}
};


namespace impl {

class GpioBase
{
protected:
	Config _config;
	bool _initialized;
	GpioBase() : _initialized(false) {}
public:
	void set_master_core(MasterCore master_core)
	{
		assert(_initialized);
		_config.master_core = master_core;
#ifdef CPU1
		GPIO_setMasterCore(_config.no, static_cast<GPIO_CoreSelect>(master_core.underlying_value()));
#endif
	}

	const Config& config() const { return _config; }
	uint32_t no() const { return _config.no; }
};

} // namespace impl


class Input : public emb::gpio::IInput, public impl::GpioBase
{
private:
	GPIO_ExternalIntNum _int_num;
public:
	Input() {}
	Input(const Config& config) { init(config); }

	void init(const Config& config)
	{
		_config = config;
		if (_config.valid)
		{
			assert(config.direction == Direction::input);
#ifdef CPU1
			GPIO_setQualificationPeriod(_config.no, _config.qual_period);
			GPIO_setQualificationMode(_config.no, static_cast<GPIO_QualificationMode>(_config.qual_mode.underlying_value()));
			GPIO_setPadConfig(_config.no, _config.type.underlying_value());
			GPIO_setPinConfig(_config.mux);
			GPIO_setDirectionMode(_config.no, GPIO_DIR_MODE_IN);
			GPIO_setMasterCore(_config.no, static_cast<GPIO_CoreSelect>(_config.master_core.underlying_value()));
#endif
			_initialized = true;
		}
	}

	virtual emb::gpio::State read() const
	{
		assert(_initialized);
		return static_cast<emb::gpio::State>(1
				- (GPIO_readPin(_config.no) ^ static_cast<uint32_t>(_config.active_state.underlying_value())));
	}

public:
#ifdef CPU1
	void set_interrupt(GPIO_ExternalIntNum intNum)
	{
		GPIO_setInterruptPin(_config.no, intNum);	// X-Bar may be configured on CPU1 only
	}
#endif
	void register_interrupt_handler(GPIO_ExternalIntNum int_num, GPIO_IntType int_type, void (*handler)(void))
	{
		_int_num = int_num;
		GPIO_setInterruptType(int_num, int_type);
		Interrupt_register(impl::pie_xint_nums[int_num], handler);
		Interrupt_enable(impl::pie_xint_nums[_int_num]);
	}

	void enable_interrupts() { GPIO_enableInterrupt(_int_num); }
	void disable_interrupts() { GPIO_disableInterrupt(_int_num); }
	void acknowledge_interrupt() { Interrupt_clearACKGroup(impl::pie_xint_groups[_int_num]); }
};


class Output : public emb::gpio::IOutput, public impl::GpioBase
{
public:
	Output() {}
	Output(const Config& config) { init(config); }

	void init(const Config& config)
	{
		_config = config;
		if (_config.valid)
		{
			assert(config.direction == Direction::output);
#ifdef CPU1
			GPIO_setPadConfig(_config.no, _config.type.underlying_value());
			//set() - is virtual, shouldn't be called in ctor
			GPIO_writePin(_config.no, 1
					- (static_cast<uint32_t>(emb::gpio::State::inactive) ^ static_cast<uint32_t>(_config.active_state.underlying_value())));
			GPIO_setPinConfig(_config.mux);
			GPIO_setDirectionMode(_config.no, GPIO_DIR_MODE_OUT);
			GPIO_setMasterCore(_config.no, static_cast<GPIO_CoreSelect>(_config.master_core.underlying_value()));
#endif
			_initialized = true;
		}
	}

	virtual emb::gpio::State read() const
	{
		assert(_initialized);
		return static_cast<emb::gpio::State>(1
				- (GPIO_readPin(_config.no) ^ static_cast<uint32_t>(_config.active_state.underlying_value())));
	}

	virtual void set(emb::gpio::State state = emb::gpio::State::active)
	{
		assert(_initialized);
		GPIO_writePin(_config.no, 1
				- (static_cast<uint32_t>(state.underlying_value()) ^ static_cast<uint32_t>(_config.active_state.underlying_value())));
	}

	virtual void reset()
	{
		assert(_initialized);
		set(emb::gpio::State::inactive);
	}

	virtual void toggle()
	{
		assert(_initialized);
		GPIO_togglePin(_config.no);
	}
};


class InputDebouncer
{
private:
	const Input _pin;
public:
	const unsigned int acq_period_ms;
	const unsigned int active_debounce_ms;
	const unsigned int inactive_debounce_ms;
private:
	const unsigned int _active_debounce_count;
	const unsigned int _inactive_debounce_count;
	unsigned int _count;
	emb::gpio::State _state;
	bool _state_changed;
public:
	InputDebouncer(const Input& pin, unsigned int acq_period_ms, unsigned int act_ms, unsigned int inact_ms)
		: _pin(pin)
		, acq_period_ms(acq_period_ms)
		, active_debounce_ms(act_ms)
		, inactive_debounce_ms(inact_ms)
		, _active_debounce_count(act_ms / acq_period_ms)
		, _inactive_debounce_count(inact_ms / acq_period_ms)
		, _state(emb::gpio::State::inactive)
		, _state_changed(false)
	{
		_count = _active_debounce_count;
	}

	void debounce()
	{
		_state_changed = false;
		emb::gpio::State raw_state = _pin.read();

		if (raw_state == _state)
		{
			if (_state == emb::gpio::State::active)
			{
				_count = _inactive_debounce_count;
			}
			else
			{
				_count = _active_debounce_count;
			}
		}
		else
		{
			if (--_count == 0)
			{
				_state = raw_state;
				_state_changed = true;
				if (_state == emb::gpio::State::active)
				{
					_count = _inactive_debounce_count;
				}
				else
				{
					_count = _active_debounce_count;
				}
			}
		}
	}

	emb::gpio::State state() const { return _state; };
	bool state_changed() const { return _state_changed; };
};


SCOPED_ENUM_DECLARE_BEGIN(DurationLoggerMode)
{
	set_reset,
	toggle
}
SCOPED_ENUM_DECLARE_END(DurationLoggerMode)


template <DurationLoggerMode::enum_type Mode = DurationLoggerMode::set_reset>
class DurationLogger
{
private:
	const uint32_t _pin;
public:
	explicit DurationLogger(const mcu::gpio::Output& pin)
		: _pin(pin.no())
	{
		if (Mode == DurationLoggerMode::set_reset)
		{
			GPIO_writePin(_pin, 1);
		}
		else
		{
			GPIO_togglePin(_pin);
			NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;
			GPIO_togglePin(_pin);
		}
	}

	~DurationLogger()
	{
		if (Mode == DurationLoggerMode::set_reset)
		{
			GPIO_writePin(_pin, 0);
		}
		else
		{
			GPIO_togglePin(_pin);
		}
	}
};

} //namespace gpio

} // namespace mcu

