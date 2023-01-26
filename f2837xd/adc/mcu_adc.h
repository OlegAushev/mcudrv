#pragma once


#include <emb_c28x/emb_array.h>
#include <emb_c28x/emb_core.h>
#include <mcu/adc_channels/mcu_adcchannels.h>
#include <mcu_c28x/f2837xd/system/mcu_system.h>
#include "driverlib.h"
#include "device.h"


namespace mcu {

namespace adc {

SCOPED_ENUM_DECLARE_BEGIN(Peripheral)
{
	adca,
	adcb,
	adcc,
	adcd
}
SCOPED_ENUM_DECLARE_END(Peripheral)


struct Config
{
	uint32_t sample_window_ns;
};


namespace impl {

struct Module
{
	uint32_t base;
};


extern const uint32_t adc_bases[4];
extern const uint16_t adc_pie_int_groups[4];


struct Channel
{
	uint32_t base;
	uint32_t result_base;
	ADC_Channel channel;
	ADC_SOCNumber soc;
	ADC_Trigger trigger;
	Channel() {}
	Channel(uint32_t base_, uint32_t result_base_, ADC_Channel channel_, ADC_SOCNumber soc_, ADC_Trigger trigger_)
		: base(base_)
		, result_base(result_base_)
		, channel(channel_)
		, soc(soc_)
		, trigger(trigger_)
	{}
};


struct Irq
{
	uint32_t base;
	ADC_IntNumber int_num;
	ADC_SOCNumber soc;
	uint32_t pie_int_num;
	Irq() {}
	Irq(uint32_t base_, ADC_IntNumber int_num_, ADC_SOCNumber soc_, uint32_t pie_int_num_)
		: base(base_)
		, int_num(int_num_)
		, soc(soc_)
		, pie_int_num(pie_int_num_)
	{}
};


void init_channels(emb::Array<impl::Channel, ChannelName::count>& channels);


void init_irqs(emb::Array<impl::Irq, IrqName::count>& irqs);

} // namespace impl


class Module : public emb::c28x::interrupt_invoker<Module>, private emb::noncopyable
{
private:
	impl::Module _module[4];
	static emb::Array<impl::Channel, ChannelName::count> _channels;
	static emb::Array<impl::Irq, IrqName::count> _irqs;
	const uint32_t sample_window_cycles;
public:
	Module(const adc::Config& config);
	void start(ChannelName channel)
	{
		ADC_forceSOC(_channels[channel.underlying_value()].base, _channels[channel.underlying_value()].soc);
	}

	uint16_t read(ChannelName channel) const
	{
		return ADC_readResult(_channels[channel.underlying_value()].result_base, _channels[channel.underlying_value()].soc);
	}

	void enable_interrupts()
	{
		for (size_t i = 0; i < IrqName::count; ++i)
		{
			Interrupt_enable(_irqs[i].pie_int_num);
		}
	}

	void disable_interrupts()
	{
		for (size_t i = 0; i < IrqName::count; ++i)
		{
			Interrupt_disable(_irqs[i].pie_int_num);
		}
	}

	void register_interrupt_handler(IrqName irq, void (*handler)(void))
	{
		Interrupt_register(_irqs[irq.underlying_value()].pie_int_num, handler);
	}

	void acknowledge_interrupt(IrqName irq)
	{
		ADC_clearInterruptStatus(_irqs[irq.underlying_value()].base, _irqs[irq.underlying_value()].int_num);
		Interrupt_clearACKGroup(impl::adc_pie_int_groups[_irqs[irq.underlying_value()].int_num]);
	}

	bool interrupt_pending(IrqName irq) const
	{
		return ADC_getInterruptStatus(_irqs[irq.underlying_value()].base, _irqs[irq.underlying_value()].int_num);
	}

	void clear_interrupt_status(IrqName irq)
	{
		ADC_clearInterruptStatus(_irqs[irq.underlying_value()].base, _irqs[irq.underlying_value()].int_num);
	}
};


class Channel
{
public:
	Module* adc;
private:
	ChannelName _channel_name;
public:
	Channel()
		: adc(Module::instance())
		, _channel_name(ChannelName::count)	// dummy write
	{}

	Channel(ChannelName channel_name)
		: adc(Module::instance())
		, _channel_name(channel_name)
	{}

	void init(ChannelName channel_name)
	{
		_channel_name = channel_name;
	}

	void start() { adc->start(_channel_name); }
	uint16_t read() const {	return adc->read(_channel_name);	}
};

} // namespace adc

} // namespace mcu

