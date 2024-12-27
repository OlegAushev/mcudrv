#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv/apm32/f4/system/system.hpp>

namespace mcu {
namespace apm32 {

__attribute__((weak)) void init_clk() {
    fatal_error("device init clock func not implemented");
}

__attribute__((weak)) void update_clk() {
    fatal_error("device update clock func not implemented");
}

void init_core(const CoreConfig& config) {
    NVIC_ConfigPriorityGroup(config.prigroup);
}

void reset_device() { NVIC_SystemReset(); }

} // namespace apm32
} // namespace mcu

#endif
#endif
