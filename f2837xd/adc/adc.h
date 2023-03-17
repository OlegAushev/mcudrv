#pragma once


#include <emblib_c28x/array.h>
#include <emblib_c28x/core.h>
#include <mcu/adc_channels/mcu_adcchannels.h>
#include <mculib_c28x/f2837xd/system/system.h>


namespace mcu {

namespace adc {

SCOPED_ENUM_DECLARE_BEGIN(Peripheral) {
    adca,
    adcb,
    adcc,
    adcd
} SCOPED_ENUM_DECLARE_END(Peripheral)


const size_t peripheral_count = 4;


struct Config {
    uint32_t sample_window_ns;
};


namespace impl {

struct Module {
    uint32_t base;
    uint32_t result_base;
    Module(uint32_t base_, uint32_t result_base_)
            : base(base_), result_base(result_base_) {}
};


extern const uint32_t adc_bases[4];
extern const uint32_t adc_result_bases[4];
extern const uint16_t adc_pie_int_groups[4];


struct Channel {
    Peripheral peripheral;
    ADC_Channel channel;
    ADC_SOCNumber soc;
    ADC_Trigger trigger;
    bool registered;
    Channel() { registered = false; }
    Channel(Peripheral peripheral_, ADC_Channel channel_, ADC_SOCNumber soc_, ADC_Trigger trigger_)
            : peripheral(peripheral_)
            , channel(channel_)
            , soc(soc_)
            , trigger(trigger_) {
        registered = false;
    }
};


struct Irq {
    Peripheral peripheral;
    ADC_IntNumber int_num;
    ADC_SOCNumber soc;
    uint32_t pie_int_num;
    bool registered;
    Irq() { registered = false; }
    Irq(Peripheral peripheral_, ADC_IntNumber int_num_, ADC_SOCNumber soc_, uint32_t pie_int_num_)
            : peripheral(peripheral_)
            , int_num(int_num_)
            , soc(soc_)
            , pie_int_num(pie_int_num_) {
        registered = false;
    }
};


void init_channels(emb::array<impl::Channel, ChannelName::count>& channels);


void init_irqs(emb::array<impl::Irq, IrqName::count>& irqs);

} // namespace impl


class Module : public emb::c28x::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
    friend class Channel;
private:
    const Peripheral _peripheral;
    impl::Module _module;
    const uint32_t sample_window_cycles;

    static emb::array<impl::Channel, ChannelName::count> _channels;
    static emb::array<impl::Irq, IrqName::count> _irqs;
    static bool _channels_and_irqs_initialized;
public:
    Module(Peripheral peripheral, const adc::Config& config);
#ifdef CPU1
    static void transfer_control_to_cpu2(Peripheral peripheral);
#endif

    void start(ChannelName channel) {
        assert(_channels[channel.underlying_value()].peripheral == _peripheral);
        ADC_forceSOC(_module.base, _channels[channel.underlying_value()].soc);
    }

    uint16_t read(ChannelName channel) const {
        assert(_channels[channel.underlying_value()].peripheral == _peripheral);
        return ADC_readResult(_module.result_base, _channels[channel.underlying_value()].soc);
    }

    void enable_interrupts() {
        for (size_t i = 0; i < _irqs.size(); ++i) {
            if (_irqs[i].peripheral == _peripheral) {
                Interrupt_enable(_irqs[i].pie_int_num);
            }
        }
    }

    void disable_interrupts() {
        for (size_t i = 0; i < _irqs.size(); ++i) {
            if (_irqs[i].peripheral == _peripheral) {
                Interrupt_disable(_irqs[i].pie_int_num);
            }
        }
    }

    void register_interrupt_handler(IrqName irq, void (*handler)(void)) {
        assert(_irqs[irq.underlying_value()].peripheral == _peripheral);
        Interrupt_register(_irqs[irq.underlying_value()].pie_int_num, handler);
    }

    void acknowledge_interrupt(IrqName irq) {
        assert(_irqs[irq.underlying_value()].peripheral == _peripheral);
        ADC_clearInterruptStatus(_module.base, _irqs[irq.underlying_value()].int_num);
        Interrupt_clearACKGroup(impl::adc_pie_int_groups[_irqs[irq.underlying_value()].int_num]);
    }

    bool interrupt_pending(IrqName irq) const {
        assert(_irqs[irq.underlying_value()].peripheral == _peripheral);
        return ADC_getInterruptStatus(_module.base, _irqs[irq.underlying_value()].int_num);
    }

    void clear_interrupt_status(IrqName irq) {
        assert(_irqs[irq.underlying_value()].peripheral == _peripheral);
        ADC_clearInterruptStatus(_module.base, _irqs[irq.underlying_value()].int_num);
    }
};


class Channel {
private:
    Module* _adc;
    ChannelName _channel;
public:
    Channel() : _adc(static_cast<Module*>(NULL)), _channel(ChannelName::count) {}
    Channel(ChannelName channel) {
        init(channel);
    }

    void init(ChannelName channel) {
        assert(Module::_channels_and_irqs_initialized);
        assert(Module::_channels[channel.underlying_value()].registered);
        _channel = channel;
        _adc = Module::instance(Module::_channels[channel.underlying_value()].peripheral.underlying_value());
    }

    void start() { _adc->start(_channel); }
    uint16_t read() const {	return _adc->read(_channel); }
    Module* adc() { return _adc; }
};

} // namespace adc

} // namespace mcu

