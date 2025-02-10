#pragma once

#ifdef MCUDRV_C28X

#include <emblib/array.hpp>
#include <emblib/core.hpp>
#include <mcu/adc_channels/adc_channels.hpp>
#include <mcudrv/c28x/f2837xd/system/system.hpp>

namespace mcu {
namespace c28x {
namespace adc {

inline float vref() { return 3.0f; }

template<typename T>
inline T nmax() { return static_cast<T>(4095); }

SCOPED_ENUM_UT_DECLARE_BEGIN(Peripheral, uint32_t) {
    adca,
    adcb,
    adcc,
    adcd
} SCOPED_ENUM_DECLARE_END(Peripheral)

const size_t peripheral_count = 4;

struct Config {
    uint32_t sample_window_ns;
    ADC_PriorityMode pri_mode;
    bool burst_mode;
    ADC_Trigger burst_trigger;
    uint16_t burst_size;
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
extern const SysCtl_CPUSelPeriphInstance adc_cpusel_instances[4];

struct Channel {
    Peripheral peripheral;
    ADC_Channel channel;
    ADC_SOCNumber soc;
    ADC_Trigger trigger;
    bool registered;
    Channel() { registered = false; }
    Channel(Peripheral peripheral_,
            ADC_Channel channel_,
            ADC_SOCNumber soc_,
            ADC_Trigger trigger_)
            : peripheral(peripheral_),
              channel(channel_),
              soc(soc_),
              trigger(trigger_) {
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
    Irq(Peripheral peripheral_,
        ADC_IntNumber int_num_,
        ADC_SOCNumber soc_,
        uint32_t pie_int_num_)
            : peripheral(peripheral_),
              int_num(int_num_),
              soc(soc_),
              pie_int_num(pie_int_num_) {
        registered = false;
    }
};

void init_channels(emb::array<impl::Channel, ChannelId::count>& channels);

void init_irqs(emb::array<impl::Irq, IrqId::count>& irqs);

} // namespace impl

class Module : public emb::singleton_array<Module, peripheral_count>,
               private emb::noncopyable {
    friend class Channel;
private:
    const Peripheral peripheral_;
    impl::Module module_;
    const uint32_t sample_window_cycles_;

    static emb::array<impl::Channel, ChannelId::count> channels_;
    static emb::array<impl::Irq, IrqId::count> irqs_;
    static bool channels_and_irqs_initialized_;
public:
    Module(Peripheral peripheral, const adc::Config& config);
#ifdef CPU1
    static void transfer_control_to_cpu2(Peripheral peripheral);
#endif
    Peripheral peripheral() const { return peripheral_; }
    uint32_t base() const { return module_.base; }

    void start(ChannelId channel) {
        assert(channels_[channel.underlying_value()].peripheral == peripheral_);
        ADC_forceSOC(module_.base, channels_[channel.underlying_value()].soc);
    }

    uint16_t read(ChannelId channel) const {
        assert(channels_[channel.underlying_value()].peripheral == peripheral_);
        return ADC_readResult(module_.result_base,
                              channels_[channel.underlying_value()].soc);
    }

    void enable_interrupts() {
        for (size_t i = 0; i < irqs_.size(); ++i) {
            if (irqs_[i].peripheral == peripheral_) {
                Interrupt_enable(irqs_[i].pie_int_num);
            }
        }
    }

    void disable_interrupts() {
        for (size_t i = 0; i < irqs_.size(); ++i) {
            if (irqs_[i].peripheral == peripheral_) {
                Interrupt_disable(irqs_[i].pie_int_num);
            }
        }
    }

    void register_interrupt_handler(IrqId irq, void (*handler)(void)) {
        assert(irqs_[irq.underlying_value()].peripheral == peripheral_);
        Interrupt_register(irqs_[irq.underlying_value()].pie_int_num, handler);
    }

    void acknowledge_interrupt(IrqId irq) {
        assert(irqs_[irq.underlying_value()].peripheral == peripheral_);
        ADC_clearInterruptStatus(module_.base, irqs_[irq.underlying_value()].int_num);
        Interrupt_clearACKGroup(impl::adc_pie_int_groups[irqs_[irq.underlying_value()].int_num]);
    }

    bool interrupt_pending(IrqId irq) const {
        assert(irqs_[irq.underlying_value()].peripheral == peripheral_);
        return ADC_getInterruptStatus(module_.base, irqs_[irq.underlying_value()].int_num);
    }

    void clear_interrupt_status(IrqId irq) {
        assert(irqs_[irq.underlying_value()].peripheral == peripheral_);
        ADC_clearInterruptStatus(module_.base, irqs_[irq.underlying_value()].int_num);
    }
};

class Channel {
private:
    Module* adc_;
    ChannelId channel_;
public:
    Channel() : adc_(static_cast<Module*>(NULL)), channel_(ChannelId::count) {}
    Channel(ChannelId channel) {
        init(channel);
    }

    void init(ChannelId ch) {
        assert(Module::channels_and_irqs_initialized_);
        assert(Module::channels_[ch.underlying_value()].registered);
        channel_ = ch;
        adc_ = Module::instance(Module::channels_[ch.underlying_value()].peripheral.underlying_value());
    }

    void start() { adc_->start(channel_); }
    uint16_t read() const {	return adc_->read(channel_); }
    Module* adc() { return adc_; }
};

} // namespace adc
} // namespace c28x
} // namespace mcu

#endif
