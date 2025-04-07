#pragma once

#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/system/system.hpp>
#include <mcudrv/generic/gpio.hpp>

#include <emblib/array.hpp>
#include <emblib/chrono.hpp>
#include <emblib/optional.hpp>

#include <assert.h>

namespace mcu {
namespace c28x {
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

struct DigitalInputConfig {
    uint32_t pin;
    uint32_t mux;
    mcu::gpio::active_state active_state;
    Type type;
    QualMode qual_mode;
    uint32_t qual_period;
    MasterCore master_core;
};

struct DigitalOutputConfig {
    uint32_t pin;
    uint32_t mux;
    mcu::gpio::active_state active_state;
    Type type;
    MasterCore master_core;
};

namespace impl {

class GpioPin {
protected:
    bool initialized_;
    uint32_t pin_;
    uint32_t mux_;
    mcu::gpio::active_state active_state_;
    GpioPin() : initialized_(false) {}
public:
    void set_master_core(MasterCore master_core) {
        assert(initialized_);
#ifdef CPU1
        GPIO_setMasterCore(pin_, static_cast<GPIO_CoreSelect>(
                master_core.underlying_value()));
#endif
    }
    uint32_t pin_no() const { return pin_; }
    uint32_t mux() const { return mux_; }
    mcu::gpio::active_state active_state() const { return active_state_; }
};

} // namespace impl

class DigitalInput : public mcu::gpio::digital_input, public impl::GpioPin {
private:
    GPIO_ExternalIntNum int_num_;
public:
    DigitalInput() {}
    DigitalInput(const DigitalInputConfig& config) { init(config); }

    void init(const DigitalInputConfig& config)	{
        pin_ = config.pin;
        mux_ = config.mux;
        active_state_ = config.active_state;
#ifdef CPU1
        GPIO_setQualificationPeriod(config.pin, config.qual_period);
        GPIO_setQualificationMode(config.pin,
                static_cast<GPIO_QualificationMode>(
                        config.qual_mode.underlying_value()));
        GPIO_setPadConfig(config.pin, config.type.underlying_value());
        GPIO_setPinConfig(config.mux);
        GPIO_setDirectionMode(config.pin, GPIO_DIR_MODE_IN);
        GPIO_setMasterCore(config.pin,
                static_cast<GPIO_CoreSelect>(
                        config.master_core.underlying_value()));
#endif
        initialized_ = true;
    }

    virtual unsigned int read_level() const {
        assert(initialized_);
        return GPIO_readPin(pin_);
    }

    virtual mcu::gpio::pin_state read() const {
        assert(initialized_);
        if (read_level() == active_state_.underlying_value()) {
            return mcu::gpio::pin_state::active;
        }
        return mcu::gpio::pin_state::inactive;
    }

public:
#ifdef CPU1
    void set_interrupt(GPIO_ExternalIntNum int_num) {
        GPIO_setInterruptPin(pin_, int_num);    // X-Bar may be configured on CPU1 only
    }
#endif
    void register_interrupt_handler(GPIO_ExternalIntNum int_num,
                                    GPIO_IntType int_type,
                                    void (*handler)(void)) {
        int_num_ = int_num;
        GPIO_setInterruptType(int_num, int_type);
        Interrupt_register(impl::pie_xint_nums[int_num], handler);
        Interrupt_enable(impl::pie_xint_nums[int_num_]);
    }

    void enable_interrupts() { GPIO_enableInterrupt(int_num_); }
    void disable_interrupts() { GPIO_disableInterrupt(int_num_); }
    void acknowledge_interrupt() {
        Interrupt_clearACKGroup(impl::pie_xint_groups[int_num_]);
    }
};

class DigitalOutput : public mcu::gpio::digital_output, public impl::GpioPin {
public:
    DigitalOutput() {}
    DigitalOutput(const DigitalOutputConfig& config,
                  mcu::gpio::pin_state init_state =
                      mcu::gpio::pin_state::inactive) {
        init(config, init_state);
    }

    void init(const DigitalOutputConfig& config,
              mcu::gpio::pin_state init_state =
                      mcu::gpio::pin_state::inactive) {
        pin_ = config.pin;
        mux_ = config.mux;
        active_state_ = config.active_state;
        initialized_ = true;
#ifdef CPU1
        GPIO_setPadConfig(config.pin, config.type.underlying_value());
        DigitalOutput::set(init_state);
        GPIO_setPinConfig(config.mux);
        GPIO_setDirectionMode(config.pin, GPIO_DIR_MODE_OUT);
        GPIO_setMasterCore(config.pin, static_cast<GPIO_CoreSelect>(
                config.master_core.underlying_value()));
#endif
    }

    virtual unsigned int read_level() const {
        assert(initialized_);
        return GPIO_readPin(pin_);
    }

    virtual void set_level(unsigned int level) {
        assert(initialized_);
        GPIO_writePin(pin_, level);
    }

    virtual mcu::gpio::pin_state read() const {
        assert(initialized_);
        if (read_level() == active_state_.underlying_value()) {
            return mcu::gpio::pin_state::active;
        }
        return mcu::gpio::pin_state::inactive;
    }

    virtual void set(mcu::gpio::pin_state s = mcu::gpio::pin_state::active) {
        assert(initialized_);
        if (s == mcu::gpio::pin_state::active) {
            set_level(active_state_.underlying_value());
        } else {
            set_level(1 - active_state_.underlying_value());
        }
    }

    virtual void reset() {
        assert(initialized_);
        set(mcu::gpio::pin_state::inactive);
    }

    virtual void toggle() {
        assert(initialized_);
        GPIO_togglePin(pin_);
    }
};

class InputDebouncer {
private:
    const DigitalInput pin_;
    const int active_debounce_count_;
    const int inactive_debounce_count_;
    int count_;
    mcu::gpio::pin_state state_;
    bool state_changed_;
public:
    InputDebouncer(const DigitalInput& pin,
                   emb::chrono::milliseconds acq_period,
                   emb::chrono::milliseconds active_debounce,
                   emb::chrono::milliseconds inactive_debounce)
            : pin_(pin),
              active_debounce_count_(active_debounce.count() / acq_period.count()),
              inactive_debounce_count_(inactive_debounce.count() / acq_period.count()),
              state_(mcu::gpio::pin_state::inactive),
              state_changed_(false) {
        count_ = active_debounce_count_;
    }

    void debounce() {
        state_changed_ = false;
        mcu::gpio::pin_state raw_state = pin_.read();

        if (raw_state == state_) {
            if (state_ == mcu::gpio::pin_state::active) {
                count_ = inactive_debounce_count_;
            } else {
                count_ = active_debounce_count_;
            }
        } else {
            if (--count_ == 0) {
                state_ = raw_state;
                state_changed_ = true;
                if (state_ == mcu::gpio::pin_state::active) {
                    count_ = inactive_debounce_count_;
                } else {
                    count_ = active_debounce_count_;
                }
            }
        }
    }

    mcu::gpio::pin_state state() const { return state_; };
    bool state_changed() const { return state_changed_; };
};

SCOPED_ENUM_DECLARE_BEGIN(DurationLoggerMode) {
    set_reset,
    toggle
} SCOPED_ENUM_DECLARE_END(DurationLoggerMode)

SCOPED_ENUM_UT_DECLARE_BEGIN(DurationLoggerChannel, uint32_t) {
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

struct DurationLoggerPinConfig {
    uint32_t pin;
    uint32_t mux;
    mcu::c28x::gpio::MasterCore core;
};

class DurationLogger {
private:
    static emb::array<emb::optional<uint32_t>, 16> pins_;
    const emb::optional<uint32_t> pin_;
    const DurationLoggerMode mode_;
public:
    static void init_channel(DurationLoggerChannel ch,
                             const DurationLoggerPinConfig config) {
        DigitalOutputConfig out_config = {
            config.pin,
            config.mux,
            mcu::gpio::active_state::high,
            mcu::c28x::gpio::Type::std,
            config.core};
        DigitalOutput out(out_config);
        pins_[ch.underlying_value()] = config.pin;
    }

    explicit DurationLogger(DurationLoggerChannel ch, DurationLoggerMode mode)
            : pin_(pins_[ch.underlying_value()])
            , mode_(mode) {
        if (!pin_.has_value()) {
            return;
        }

        if (mode_ == DurationLoggerMode::set_reset) {
            GPIO_writePin(*pin_, 1);
        } else {
            GPIO_togglePin(*pin_);
            NOP; NOP; NOP; NOP; NOP; NOP; NOP; NOP;
            GPIO_togglePin(*pin_);
        }
    }

    ~DurationLogger() {
        if (!pin_.has_value()) {
            return;
        }

        if (mode_ == DurationLoggerMode::set_reset) {
            GPIO_writePin(*pin_, 0);
        } else {
            GPIO_togglePin(*pin_);
        }
    }
};

} //namespace gpio
} // namespace c28x
} // namespace mcu

#endif
