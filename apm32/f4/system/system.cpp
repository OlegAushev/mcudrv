#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/chrono/chrono.h>


namespace mcu {


void init(const CoreConfig& config) {
    NVIC_ConfigPriorityGroup(config.prigroup);
}


void delay(std::chrono::milliseconds delay) {
    auto start = chrono::system_clock::now();
    while ((chrono::system_clock::now() - start) <= delay) {
        // wait
    }
}


void fatal_error(const char* hint, int code) {
    emb::fatal_error(hint, code);
}


} // namespace mcu


#endif
#endif
