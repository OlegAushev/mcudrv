#pragma once

#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/system/system.hpp>
#include <emblib/core.hpp>

namespace mcu {
namespace c28x {
namespace dac {

SCOPED_ENUM_UT_DECLARE_BEGIN(Peripheral, uint32_t) {
    daca,
    dacb,
    dacc
} SCOPED_ENUM_DECLARE_END(Peripheral)

const size_t peripheral_count = 3;

namespace impl {

struct Module {
    uint32_t base;
    Module(uint32_t base_) : base(base_) {}
};

extern const uint32_t dac_bases[3];

} // namespace impl

class Input {
private:
    uint16_t tag_ : 4;
    uint16_t value_ : 12;
public:
    Input() : tag_(0), value_(0) {}
    explicit Input(uint16_t value) : tag_(0), value_(value & 0x0FFF) {}
    Input(uint16_t value, Peripheral peripheral)
            : tag_(static_cast<uint16_t>(peripheral.underlying_value()))
            , value_(value & 0x0FFF) {}

    uint16_t get() const { return value_; }
    uint16_t tag() const { return tag_; }
};

class Module : public emb::singleton_array<Module, peripheral_count>,
               private emb::noncopyable {
private:
    const Peripheral peripheral_;
    impl::Module module_;
public:
    Module(Peripheral peripheral);
    Peripheral peripheral() const { return peripheral_; }
    uint32_t base() const { return module_.base; }
    void convert(Input input) { DAC_setShadowValue(module_.base, input.get()); }
};

} // namespace dac
} // namespace c28x
} // namespace mcu

#endif
