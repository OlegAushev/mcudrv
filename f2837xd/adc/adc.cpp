#include <mculib_c28x/f2837xd/adc/adc.h>


namespace mcu {

namespace adc {

const uint32_t impl::adc_bases[4] = {ADCA_BASE, ADCB_BASE, ADCC_BASE, ADCD_BASE};
const uint32_t impl::adc_result_bases[4]= {ADCARESULT_BASE, ADCBRESULT_BASE, ADCCRESULT_BASE, ADCDRESULT_BASE};
const uint16_t impl::adc_pie_int_groups[4] = {INTERRUPT_ACK_GROUP1, INTERRUPT_ACK_GROUP10,
        INTERRUPT_ACK_GROUP10, INTERRUPT_ACK_GROUP10};


emb::array<impl::Channel, ChannelId::count> Module::_channels;
emb::array<impl::Irq, IrqId::count> Module::_irqs;
bool Module::_channels_and_irqs_initialized = false;


Module::Module(Peripheral peripheral, const adc::Config& config)
        : emb::c28x::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
        , _peripheral(peripheral)
        , _module(impl::adc_bases[peripheral.underlying_value()], impl::adc_result_bases[peripheral.underlying_value()])
        , sample_window_cycles(config.sample_window_ns / (1000000000 / mcu::sysclk_freq())) {
    if (!_channels_and_irqs_initialized) {
        impl::init_channels(_channels);
        impl::init_irqs(_irqs);
        _channels_and_irqs_initialized = true;
    }

    ADC_setPrescaler(_module.base, ADC_CLK_DIV_4_0);        // fclk(adc)max = 50 MHz
    ADC_setMode(_module.base, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
    ADC_setInterruptPulseMode(_module.base, ADC_PULSE_END_OF_CONV);
    ADC_enableConverter(_module.base);
    ADC_setSOCPriority(_module.base, ADC_PRI_ALL_HIPRI);    // SOCs at high priority - easier to control order
    mcu::delay(emb::chrono::microseconds(1000));            // delay for power-up

    // Configure SOCs
    // For 12-bit resolution, a sampling window of (5 x sample_window_cycles)ns
    // at a 200MHz SYSCLK rate will be used
    for (size_t i = 0; i < _channels.size(); ++i) {
        if (_channels[i].peripheral == _peripheral) {
            ADC_setupSOC(_module.base, _channels[i].soc, _channels[i].trigger, _channels[i].channel, sample_window_cycles);
            _channels[i].registered = true;
        }

    }

    // Interrupt config
    for (size_t i = 0; i < _irqs.size(); ++i) {
        if (_irqs[i].peripheral == _peripheral) {
            ADC_setInterruptSource(_module.base, _irqs[i].int_num, _irqs[i].soc);
            ADC_enableInterrupt(_module.base, _irqs[i].int_num);
            ADC_clearInterruptStatus(_module.base, _irqs[i].int_num);
            _irqs[i].registered = true;
        }
    }
}


#ifdef CPU1
void Module::transfer_control_to_cpu2(Peripheral peripheral) {
    SysCtl_CPUSelPeriphInstance peripheral_;
    switch (peripheral.native_value()) {
    case Peripheral::adca:
        peripheral_ = SYSCTL_CPUSEL_ADCA;
        break;
    case Peripheral::adcb:
        peripheral_ = SYSCTL_CPUSEL_ADCB;
        break;
    case Peripheral::adcc:
        peripheral_ = SYSCTL_CPUSEL_ADCC;
        break;
    case Peripheral::adcd:
        peripheral_ = SYSCTL_CPUSEL_ADCD;
        break;
    }

    SysCtl_selectCPUForPeripheralInstance(peripheral_ ,SYSCTL_CPUSEL_CPU2);
}
#endif

} // namespace adc

} // namespace mcu

