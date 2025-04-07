#pragma once

#include <emblib/core.hpp>

namespace mcu {
namespace gpio {

#if defined(MCUDRV_C28X)

SCOPED_ENUM_UT_DECLARE_BEGIN(active_state, unsigned int) {
    low = 0,
    high = 1
} SCOPED_ENUM_DECLARE_END(active_state)

SCOPED_ENUM_UT_DECLARE_BEGIN(pin_state, unsigned int) {
    inactive = 0,
    active = 1
} SCOPED_ENUM_DECLARE_END(pin_state)

class digital_input {
public:
    digital_input() {}
    virtual ~digital_input() {}

    virtual pin_state read() const = 0;
    virtual unsigned int read_level() const = 0;
};

class digital_output {
public:
    digital_output() {}
    virtual ~digital_output() {}

    virtual pin_state read() const = 0;
    virtual void set(pin_state s = pin_state::active) = 0;
    virtual void reset() = 0;
    virtual void toggle() = 0;
    virtual unsigned int read_level() const = 0;
    virtual void set_level(unsigned int level) = 0;
};

#else

enum class active_state : unsigned int {
    low = 0,
    high = 1
};

enum class pin_state : unsigned int {
    inactive = 0,
    active = 1
};

class input_pin {
public:
    input_pin() = default;
    virtual ~input_pin() = default;

    virtual pin_state read() const = 0;
    virtual unsigned int read_level() const = 0;
};

class output_pin {
public:
    output_pin() = default;
    virtual ~output_pin() = default;

    virtual pin_state read() const = 0;
    virtual void set(pin_state s = pin_state::active) = 0;
    virtual void reset() = 0;
    virtual void toggle() = 0;
    virtual unsigned int read_level() const = 0;
    virtual void set_level(unsigned int level) = 0;
};

#endif

} // namespace gpio
} // namespace mcu
