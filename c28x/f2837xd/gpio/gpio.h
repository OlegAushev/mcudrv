#pragma once


#ifdef MCUDRV_C28X


#include "../system/system.h"
#include <emblib/interfaces/gpio.h>
#include <emblib/chrono.h>
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


struct Config {
    bool valid;
    uint32_t no;
    uint32_t mux;
    Direction direction;
    emb::gpio::active_pin_state actstate;
    Type type;
    QualMode qual_mode;
    uint32_t qual_period;
    MasterCore master_core;

    Config() : valid(false) {}

    Config(uint32_t no_, uint32_t mux_, Direction direction_, emb::gpio::active_pin_state actstate_,
            Type type_, QualMode qual_mode_, uint32_t qual_period_,
            MasterCore master_core_ = MasterCore::cpu1)
            : valid(true)
            , no(no_)
            , mux(mux_)
            , direction(direction_)
            , actstate(actstate_)
            , type(type_)
            , qual_mode(qual_mode_)
            , qual_period(qual_period_)
            , master_core(master_core_) {
    }

    Config(uint32_t no_, uint32_t mux_)
            : valid(false)
            , no(no_)
            , mux(mux_) {
    }

    Config(traits::unused) : valid(false) {}
};


namespace impl {


class GpioPin {
protected:
    Config _cfg;
    bool _initialized;
    GpioPin() : _initialized(false) {}
public:
    void set_master_core(MasterCore master_core) {
        assert(_initialized);
        _cfg.master_core = master_core;
#ifdef CPU1
        GPIO_setMasterCore(_cfg.no, static_cast<GPIO_CoreSelect>(master_core.underlying_value()));
#endif
    }

    const Config& config() const { return _cfg; }
    uint32_t no() const { return _cfg.no; }
};


} // namespace impl


class InputPin : public emb::gpio::input_pin, public impl::GpioPin {
private:
    GPIO_ExternalIntNum _int_num;
public:
    InputPin() {}
    InputPin(const Config& config) { init(config); }

    void init(const Config& config)	{
        _cfg = config;
        if (_cfg.valid) {
            assert(config.direction == Direction::input);
#ifdef CPU1
            GPIO_setQualificationPeriod(_cfg.no, _cfg.qual_period);
            GPIO_setQualificationMode(_cfg.no, static_cast<GPIO_QualificationMode>(_cfg.qual_mode.underlying_value()));
            GPIO_setPadConfig(_cfg.no, _cfg.type.underlying_value());
            GPIO_setPinConfig(_cfg.mux);
            GPIO_setDirectionMode(_cfg.no, GPIO_DIR_MODE_IN);
            GPIO_setMasterCore(_cfg.no, static_cast<GPIO_CoreSelect>(_cfg.master_core.underlying_value()));
#endif
            _initialized = true;
        }
    }

    virtual unsigned int read_level() const {
        assert(_initialized);
        return GPIO_readPin(_cfg.no);
    }

    virtual emb::gpio::pin_state read() const {
        assert(_initialized);
        if (read_level() == _cfg.actstate.underlying_value()) {
            return emb::gpio::pin_state::active;
        }
        return emb::gpio::pin_state::inactive;
    }

public:
#ifdef CPU1
    void set_interrupt(GPIO_ExternalIntNum intNum) {
        GPIO_setInterruptPin(_cfg.no, intNum);   // X-Bar may be configured on CPU1 only
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
    OutputPin(const Config& config) { init(config); }

    void init(const Config& config) {
        _cfg = config;
        if (_cfg.valid) {
            assert(config.direction == Direction::output);
#ifdef CPU1
            GPIO_setPadConfig(_cfg.no, _cfg.type.underlying_value());
            //set() - is virtual, shouldn't be called in ctor
            GPIO_writePin(_cfg.no,
                    1 - (static_cast<uint32_t>(emb::gpio::pin_state::inactive) ^ static_cast<uint32_t>(_cfg.actstate.underlying_value())));
            GPIO_setPinConfig(_cfg.mux);
            GPIO_setDirectionMode(_cfg.no, GPIO_DIR_MODE_OUT);
            GPIO_setMasterCore(_cfg.no, static_cast<GPIO_CoreSelect>(_cfg.master_core.underlying_value()));
#endif
            _initialized = true;
        }
    }

    virtual unsigned int read_level() const {
        assert(_initialized);
        return GPIO_readPin(_cfg.no);
    }

    virtual void set_level(unsigned int level) {
        assert(_initialized);
        GPIO_writePin(_cfg.no, level);
    }

    virtual emb::gpio::pin_state read() const {
        assert(_initialized);
        if (read_level() == _cfg.actstate.underlying_value()) {
            return emb::gpio::pin_state::active;
        }
        return emb::gpio::pin_state::inactive;
    }

    virtual void set(emb::gpio::pin_state s = emb::gpio::pin_state::active) {
        assert(_initialized);
        if (s == emb::gpio::pin_state::active) {
            set_level(_cfg.actstate.underlying_value());
        } else {
            set_level(1 - _cfg.actstate.underlying_value());
        }
    }

    virtual void reset() {
        assert(_initialized);
        set(emb::gpio::pin_state::inactive);
    }

    virtual void toggle() {
        assert(_initialized);
        GPIO_togglePin(_cfg.no);
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


template <DurationLoggerMode::enum_type Mode = DurationLoggerMode::set_reset>
class DurationLogger {
private:
    const uint32_t _pin;
public:
    explicit DurationLogger(const mcu::gpio::OutputPin& pin) : _pin(pin.no()) {
        if (Mode == DurationLoggerMode::set_reset) {
            GPIO_writePin(_pin, 1);
        } else {
            GPIO_togglePin(_pin);
            NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;
            GPIO_togglePin(_pin);
        }
    }

    explicit DurationLogger(uint32_t pin_num) : _pin(pin_num) {
        if (Mode == DurationLoggerMode::set_reset) {
            GPIO_writePin(_pin, 1);
        } else {
            GPIO_togglePin(_pin);
            NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;
            GPIO_togglePin(_pin);
        }
    }

    ~DurationLogger() {
        if (Mode == DurationLoggerMode::set_reset) {
            GPIO_writePin(_pin, 0);
        } else {
            GPIO_togglePin(_pin);
        }
    }
};


} //namespace gpio


} // namespace mcu


#endif
