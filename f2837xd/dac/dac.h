#pragma once


#include "../system/system.h"
#include <emblib_c28x/core.h>


namespace mcu {

namespace dac {

SCOPED_ENUM_DECLARE_BEGIN(Peripheral)
{
	daca,
	dacb,
	dacc
}
SCOPED_ENUM_DECLARE_END(Peripheral)


const size_t peripheral_count = 3;


namespace impl {

struct Module
{
	uint32_t base;
	Module(uint32_t base_) : base(base_) {}
};


extern const uint32_t dac_bases[3];

} // namespace impl


class Input
{
private:
	uint16_t _tag : 4;
	uint16_t _value : 12;
public:
	Input()
		: _tag(0)
		, _value(0)
	{}

	explicit Input(uint16_t value)
		: _tag(0)
		, _value(value & 0x0FFF)
	{}

	Input(uint16_t value, Peripheral peripheral)
		: _tag(static_cast<uint16_t>(peripheral.underlying_value()))
		, _value(value & 0x0FFF)
	{}

	uint16_t get() const { return _value; }
	uint16_t tag() const { return _tag; }
};


class Module : public emb::c28x::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable
{
private:
	const Peripheral _peripheral;
	impl::Module _module;
public:
	Module(Peripheral peripheral);
	void convert(Input input) { DAC_setShadowValue(_module.base, input.get()); }
};

} // namespace dac

} // namespace mcu

