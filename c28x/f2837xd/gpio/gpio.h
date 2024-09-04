#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include <emblib/interfaces/gpio.h>
#include <emblib/array.h>
#include <emblib/chrono.h>
#include <emblib/optional.h>
#include <assert.h>


namespace mcu {


namespace gpio {


SCOPED_ENUM_DECLARE_BEGIN(Type) {
    std = GPIO_PIN_TYPE_STD,
    pullup = GPIO_PIN_TYPE_PULLUP,
    invert = GPIO_PIN_TYPE_INVERT,
    open_drain = GPIO_PIN_TYPE_OD
} SCOPED_ENUM_DECLARE_END(Type)


SCOPED_ENUM_DECLARE_BEGIN(Direction) {
    input = GPIO_DIR_MODE_IN,
    output = GPIO_DIR_MODE_OUT
} SCOPED_ENUM_DECLARE_END(Direction)


SCOPED_ENUM_DECLARE_BEGIN(QualMode) {
    sync = GPIO_QUAL_SYNC,
    sample3 = GPIO_QUAL_3SAMPLE,
    sample6 = GPIO_QUAL_6SAMPLE,
    async = GPIO_QUAL_ASYNC
} SCOPED_ENUM_DECLARE_END(QualMode)


SCOPED_ENUM_DECLARE_BEGIN(MasterCore) {
    cpu1 = GPIO_CORE_CPU1,
    cpu1_cla1 = GPIO_CORE_CPU1_CLA1,
    cpu2 = GPIO_CORE_CPU2,
    cpu2_cla1 = GPIO_CORE_CPU2_CLA1
} SCOPED_ENUM_DECLARE_END(MasterCore)


namespace impl {


extern const uint32_t pie_xint_nums[5];
extern const uint16_t pie_xint_groups[5];


} // namespace impl


struct PinConfig {
    uint32_t pin;
    uint32_t mux;
    Direction direction;
    emb::gpio::active_pin_state actstate;
    Type type;
    QualMode qual_mode;
    uint32_t qual_period;
    MasterCore master_core;
};


namespace impl {


class GpioPin {
protected:
    bool _initialized;
    uint32_t _pin;
    emb::gpio::active_pin_state _actstate;
    GpioPin() : _initialized(false) {}
public:
    void set_master_core(MasterCore master_core) {
        assert(_initialized);
#ifdef CPU1
        GPIO_setMasterCore(_pin, static_cast<GPIO_CoreSelect>(master_core.underlying_value()));
#endif
    }
    uint32_t pin_no() const { return _pin; }
};


} // namespace impl


class InputPin : public emb::gpio::input_pin, public impl::GpioPin {
private:
    GPIO_ExternalIntNum _int_num;
public:
    InputPin() {}
    InputPin(const PinConfig& config) { init(config); }

    void init(const PinConfig& config)	{
        assert(config.direction == Direction::input);
        _pin = config.pin;
        _actstate = config.actstate;
#ifdef CPU1
        GPIO_setQualificationPeriod(config.pin, config.qual_period);
        GPIO_setQualificationMode(config.pin, static_cast<GPIO_QualificationMode>(config.qual_mode.underlying_value()));
        GPIO_setPadConfig(config.pin, config.type.underlying_value());
        GPIO_setPinConfig(config.mux);
        GPIO_setDirectionMode(config.pin, GPIO_DIR_MODE_IN);
        GPIO_setMasterCore(config.pin, static_cast<GPIO_CoreSelect>(config.master_core.underlying_value()));
#endif
        _initialized = true;
    }

    virtual unsigned int read_level() const {
        assert(_initialized);
        return GPIO_readPin(_pin);
    }

    virtual emb::gpio::pin_state read() const {
        assert(_initialized);
        if (read_level() == _actstate.underlying_value()) {
            return emb::gpio::pin_state::active;
        }
        return emb::gpio::pin_state::inactive;
    }

public:
#ifdef CPU1
    void set_interrupt(GPIO_ExternalIntNum int_num) {
        GPIO_setInterruptPin(_pin, int_num);    // X-Bar may be configured on CPU1 only
    }
#endif
    void register_interrupt_handler(GPIO_ExternalIntNum int_num, GPIO_IntType int_type, void (*handler)(void)) {
        _int_num = int_num;
        GPIO_setInterruptType(int_num, int_type);
        Interrupt_register(impl::pie_xint_nums[int_num], handler);
        Interrupt_enable(impl::pie_xint_nums[_int_num]);
    }

    void enable_interrupts() { GPIO_enableInterrupt(_int_num); }
    void disable_interrupts() { GPIO_disableInterrupt(_int_num); }
    void acknowledge_interrupt() { Interrupt_clearACKGroup(impl::pie_xint_groups[_int_num]); }
};


class OutputPin : public emb::gpio::output_pin, public impl::GpioPin {
public:
    OutputPin() {}
    OutputPin(const PinConfig& config) { init(config); }

    void init(const PinConfig& config) {
        assert(config.direction == Direction::output);
        _pin = config.pin;
        _actstate = config.actstate;
#ifdef CPU1
        GPIO_setPadConfig(config.pin, config.type.underlying_value());
        //set() - is virtual, shouldn't be called in ctor
        GPIO_writePin(config.pin,
                1 - (static_cast<uint32_t>(emb::gpio::pin_state::inactive) ^ static_cast<uint32_t>(config.actstate.underlying_value())));
        GPIO_setPinConfig(config.mux);
        GPIO_setDirectionMode(config.pin, GPIO_DIR_MODE_OUT);
        GPIO_setMasterCore(config.pin, static_cast<GPIO_CoreSelect>(config.master_core.underlying_value()));
#endif
        _initialized = true;
    }

    virtual unsigned int read_level() const {
        assert(_initialized);
        return GPIO_readPin(_pin);
    }

    virtual void set_level(unsigned int level) {
        assert(_initialized);
        GPIO_writePin(_pin, level);
    }

    virtual emb::gpio::pin_state read() const {
        assert(_initialized);
        if (read_level() == _actstate.underlying_value()) {
            return emb::gpio::pin_state::active;
        }
        return emb::gpio::pin_state::inactive;
    }

    virtual void set(emb::gpio::pin_state s = emb::gpio::pin_state::active) {
        assert(_initialized);
        if (s == emb::gpio::pin_state::active) {
            set_level(_actstate.underlying_value());
        } else {
            set_level(1 - _actstate.underlying_value());
        }
    }

    virtual void reset() {
        assert(_initialized);
        set(emb::gpio::pin_state::inactive);
    }

    virtual void toggle() {
        assert(_initialized);
        GPIO_togglePin(_pin);
    }
};


class InputDebouncer {
private:
    const InputPin _pin;
    const int _active_debounce_count;
    const int _inactive_debounce_count;
    int _count;
    emb::gpio::pin_state _state;
    bool _state_changed;
public:
    InputDebouncer(const InputPin& pin, emb::chrono::milliseconds acq_period,
                    emb::chrono::milliseconds active_debounce, emb::chrono::milliseconds inactive_debounce)
            : _pin(pin)
            , _active_debounce_count(active_debounce.count() / acq_period.count())
            , _inactive_debounce_count(inactive_debounce.count() / acq_period.count())
            , _state(emb::gpio::pin_state::inactive)
            , _state_changed(false) {
        _count = _active_debounce_count;
    }

    void debounce() {
        _state_changed = false;
        emb::gpio::pin_state raw_state = _pin.read();

        if (raw_state == _state) {
            if (_state == emb::gpio::pin_state::active) {
                _count = _inactive_debounce_count;
            } else {
                _count = _active_debounce_count;
            }
        } else {
            if (--_count == 0) {
                _state = raw_state;
                _state_changed = true;
                if (_state == emb::gpio::pin_state::active) {
                    _count = _inactive_debounce_count;
                } else {
                    _count = _active_debounce_count;
                }
            }
        }
    }

    emb::gpio::pin_state state() const { return _state; };
    bool state_changed() const { return _state_changed; };
};


SCOPED_ENUM_DECLARE_BEGIN(DurationLoggerMode) {
    set_reset,
    toggle
} SCOPED_ENUM_DECLARE_END(DurationLoggerMode)


SCOPED_ENUM_DECLARE_BEGIN(DurationLoggerChannel) {
    channel0,
    channel1,
    channel2,
    channel3,
    channel4,
    channel5,
    channel6,
    channel7,
    channel8,
    channel9,
    channel10,
    channel11,
    channel12,
    channel13,
    channel14,
    channel15,
} SCOPED_ENUM_DECLARE_END(DurationLoggerChannel)


struct DurationLoggerPinConfig { uint32_t pin; uint32_t mux; mcu::gpio::MasterCore core; };


class DurationLogger {
private:
    static emb::array<emb::optional<uint32_t>, 16> _pins;
    const emb::optional<uint32_t> _pin;
    const DurationLoggerMode _mode;
public:
    static void init_channel(DurationLoggerChannel ch, const DurationLoggerPinConfig config) {
        PinConfig out_config = {config.pin, config.mux, mcu::gpio::Direction::output, emb::gpio::active_pin_state::high,
                             mcu::gpio::Type::std, mcu::gpio::QualMode::sync, 1, config.core};
        OutputPin out(out_config);
        _pins[ch.underlying_value()] = config.pin;
    }

    explicit DurationLogger(DurationLoggerChannel ch, DurationLoggerMode mode)
            : _pin(_pins[ch.underlying_value()])
            , _mode(mode) {
        if (!_pin.has_value()) {
            return;
        }

        if (_mode == DurationLoggerMode::set_reset) {
            GPIO_writePin(*_pin, 1);
        } else {
            GPIO_togglePin(*_pin);
            NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;
            GPIO_togglePin(*_pin);
        }
    }

    ~DurationLogger() {
        if (!_pin.has_value()) {
            return;
        }

        if (_mode == DurationLoggerMode::set_reset) {
            GPIO_writePin(*_pin, 0);
        } else {
            GPIO_togglePin(*_pin);
        }
    }
};


} //namespace gpio


} // namespace mcu


#endif
