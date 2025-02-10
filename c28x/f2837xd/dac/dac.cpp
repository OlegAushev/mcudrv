#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/dac/dac.hpp>
#include <mcudrv/c28x/f2837xd/chrono/chrono.hpp>

namespace mcu {
namespace c28x {
namespace dac {

const uint32_t impl::dac_bases[3] = {DACA_BASE, DACB_BASE, DACC_BASE};

Module::Module(Peripheral peripheral)
        : emb::singleton_array<Module, peripheral_count>(
                this, peripheral.underlying_value()),
          peripheral_(peripheral),
          module_(impl::dac_bases[peripheral.underlying_value()]) {
    DAC_setReferenceVoltage(module_.base, DAC_REF_ADC_VREFHI);
    DAC_enableOutput(module_.base);
    DAC_setShadowValue(module_.base, 0);
    chrono::delay(emb::chrono::microseconds(10));  // delay for buffered DAC to power up
}

} // namespace dac
} // namespace c28x
} // namespace mcu

#endif
