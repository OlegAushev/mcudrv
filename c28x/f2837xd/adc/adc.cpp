#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/adc/adc.hpp>
#include <mcudrv/c28x/f2837xd/chrono/chrono.hpp>

namespace mcu {
namespace c28x {
namespace adc {

const uint32_t impl::adc_bases[4] = {ADCA_BASE,
                                     ADCB_BASE,
                                     ADCC_BASE,
                                     ADCD_BASE};

const uint32_t impl::adc_result_bases[4]= {ADCARESULT_BASE,
                                           ADCBRESULT_BASE,
                                           ADCCRESULT_BASE,
                                           ADCDRESULT_BASE};

const uint16_t impl::adc_pie_int_groups[4] = {INTERRUPT_ACK_GROUP1,
                                              INTERRUPT_ACK_GROUP10,
                                              INTERRUPT_ACK_GROUP10,
                                              INTERRUPT_ACK_GROUP10};

const SysCtl_CPUSelPeriphInstance impl::adc_cpusel_instances[4] = {
        SYSCTL_CPUSEL_ADCA,
        SYSCTL_CPUSEL_ADCB,
        SYSCTL_CPUSEL_ADCC,
        SYSCTL_CPUSEL_ADCD};

emb::array<impl::Channel, ChannelId::count> Module::channels_;
emb::array<impl::Irq, IrqId::count> Module::irqs_;
bool Module::channels_and_irqs_initialized_ = false;


Module::Module(Peripheral peripheral, const adc::Config& config)
        : emb::singleton_array<Module, peripheral_count>(
                this, peripheral.underlying_value()),
          peripheral_(peripheral),
          module_(impl::adc_bases[peripheral.underlying_value()],
                  impl::adc_result_bases[peripheral.underlying_value()]),
          sample_window_cycles_(config.sample_window_ns /
                               (1000000000 / mcu::c28x::sysclk_freq())) {
    if (!channels_and_irqs_initialized_) {
        impl::init_channels(channels_);
        impl::init_irqs(irqs_);
        channels_and_irqs_initialized_ = true;
    }

    ADC_setPrescaler(module_.base, ADC_CLK_DIV_4_0);    // fclk(adc)max = 50 MHz
    ADC_setMode(module_.base, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(module_.base, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(module_.base);
    ADC_setSOCPriority(module_.base, config.pri_mode);
    if (config.burst_mode) {
        ADC_setBurstModeConfig(module_.base,
                               config.burst_trigger,
                               config.burst_size);
        ADC_enableBurstMode(module_.base);
    }

    // Delay for power-up
    mcu::c28x::chrono::delay(emb::chrono::microseconds(1000));

    // Configure SOCs
    // For 12-bit resolution, a sampling window of (5 x sample_window_cycles)ns
    // at a 200MHz SYSCLK rate will be used
    for (size_t i = 0; i < channels_.size(); ++i) {
        if (channels_[i].peripheral == peripheral_) {
            ADC_setupSOC(module_.base,
                         channels_[i].soc,
                         channels_[i].trigger,
                         channels_[i].channel,
                         sample_window_cycles_);
            channels_[i].registered = true;
        }

    }

    // Interrupt config
    for (size_t i = 0; i < irqs_.size(); ++i) {
        if (irqs_[i].peripheral == peripheral_) {
            ADC_setInterruptSource(module_.base,
                                   irqs_[i].int_num,
                                   irqs_[i].soc);
            ADC_enableInterrupt(module_.base, irqs_[i].int_num);
            ADC_clearInterruptStatus(module_.base, irqs_[i].int_num);
            irqs_[i].registered = true;
        }
    }
}

#ifdef CPU1
void Module::transfer_control_to_cpu2(Peripheral peripheral) {
    SysCtl_selectCPUForPeripheralInstance(
            impl::adc_cpusel_instances[peripheral.underlying_value()],
            SYSCTL_CPUSEL_CPU2);
}
#endif

} // namespace adc
} // namespace c28x
} // namespace mcu

#endif
