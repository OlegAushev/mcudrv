#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/dac/dac.h>
#include <mcudrv/c28x/f2837xd/chrono/chrono.h>


namespace mcu {

namespace c28x {

namespace dac {


const uint32_t impl::dac_bases[3] = {DACA_BASE, DACB_BASE, DACC_BASE};


Module::Module(Peripheral peripheral)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
        , _peripheral(peripheral)
        , _module(impl::dac_bases[peripheral.underlying_value()]) {
    DAC_setReferenceVoltage(_module.base, DAC_REF_ADC_VREFHI);
    DAC_enableOutput(_module.base);
    DAC_setShadowValue(_module.base, 0);
    chrono::delay(emb::chrono::microseconds(10));  // Delay for buffered DAC to power up
}


} // namespace dac

} // namespace c28x

} // namespace mcu


#endif
