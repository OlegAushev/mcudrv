#pragma once


#ifdef MCUDRV_C28X


#include "../../c28x_base.h"
#include <emblib/core.h>
#include <emblib/chrono.h>
#include <driverlib.h>
#include <device.h>
#include <F28x_Project.h>


namespace mcu {


namespace traits {
struct unused{};
}


inline void init_device() {
#ifdef CPU1
    Device_init();                  // Initialize device clock and peripherals
    Device_initGPIO();              // Disable pin locks and enable internal pull-ups
    Interrupt_initModule();         // Initialize PIE and clear PIE registers. Disable CPU interrupts
    Interrupt_initVectorTable();    // Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR)
#endif
#ifdef CPU2
    Device_init();
    Interrupt_initModule();
    Interrupt_initVectorTable();
#endif
}


#ifdef CPU1
inline void boot_cpu2() { Device_bootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH); }
#endif


inline void delay(emb::chrono::nanoseconds ns) {
    const uint32_t sysclkCycle_ns = 1000000000 / DEVICE_SYSCLK_FREQ;
    const uint32_t delayLoop_ns = 5 * sysclkCycle_ns;
    const uint32_t delayOverhead_ns = 9 * sysclkCycle_ns;

    if (ns.count() < delayLoop_ns + delayOverhead_ns) {
        SysCtl_delay(1);
    } else {
        SysCtl_delay((ns.count() - delayOverhead_ns) / delayLoop_ns);
    }
}


inline void delay(emb::chrono::microseconds us) {
    DEVICE_DELAY_US(us.count());
}


inline void delay(emb::chrono::milliseconds ms) {
    delay(emb::chrono::duration_cast<emb::chrono::microseconds>(ms));
}


inline void enable_maskable_interrupts() { EINT; }
inline void disable_maskable_interrupts() { DINT; }
inline void enable_debug_events() { ERTM; }
inline void disable_debug_events() { DRTM; }
inline void reset_device() { SysCtl_resetDevice(); }


class critical_section {
private:
    uint16_t int_status;
public:
    critical_section() {
        int_status = __disable_interrupts();
    }

    ~critical_section() {
        if(!(int_status & 0x1)) { EINT; }
        if(!(int_status & 0x2)) { ERTM;	}
    }
};


inline uint32_t sysclk_freq() { return DEVICE_SYSCLK_FREQ; }


#define INVOKE_USER1_INTERRUPT() __asm(" TRAP #20")
#define INVOKE_USER2_INTERRUPT() __asm(" TRAP #21")
#define INVOKE_USER3_INTERRUPT() __asm(" TRAP #22")


} // namespace mcu


#endif
