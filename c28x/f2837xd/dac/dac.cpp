#ifdef MCUDRV_C28X


#include <mculib_c28x/f2837xd/dac/dac.h>


namespace mcu {


namespace dac {


const uint32_t impl::dac_bases[3] = {DACA_BASE, DACB_BASE, DACC_BASE};


Module::Module(Peripheral peripheral)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
        , _peripheral(peripheral)
        , _module(impl::dac_bases[peripheral.underlying_value()]) {
    DAC_setReferenceVoltage(_module.base, DAC_REF_ADC_VREFHI);
    DAC_enableOutput(_module.base);
    DAC_setShadowValue(_module.base, 0);
    mcu::delay(emb::chrono::microseconds(10));  // Delay for buffered DAC to power up
}


} // namespace dac


} // namespace mcu


#endif
