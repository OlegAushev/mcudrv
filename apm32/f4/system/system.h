#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/apm32f4_base.h>
#include <apm32f4xx.h>
#include <algorithm>
#include <chrono>


namespace mcu {


void initialize_device_clock();


inline uint32_t core_clk_freq() { return SystemCoreClock; }


struct CoreConfig {
    NVIC_PRIORITY_GROUP_T prigroup;
};


void initialize(const CoreConfig& config);


void reset_device();


void delay(std::chrono::milliseconds delay);


void fatal_error();


void fatal_error(const char* hint, int code = 0);


class IrqPriority {
    friend void initialize(const CoreConfig&);
private:
#ifdef SILICON_REVISION_A
    static inline uint8_t _preempt_priorities{8};
#else
    static inline uint8_t _preempt_priorities{16};
#endif
    static inline uint8_t _sub_priorities{0}; 

    uint8_t _preempt_pri;
    uint8_t _sub_pri;
public:
    explicit IrqPriority(uint8_t preempt_pri, uint8_t sub_pri)
            : _preempt_pri(std::clamp(preempt_pri, uint8_t(0), _preempt_priorities))
            , _sub_pri(std::clamp(sub_pri, uint8_t(0), _sub_priorities))
    {
        assert(preempt_pri <= _preempt_priorities);
        assert(sub_pri <= _sub_priorities);
    }

    uint8_t preempt_pri() const { return _preempt_pri; }
    uint8_t sub_pri() const { return _sub_pri; }
};


inline void set_irq_priority(IRQn_Type irqn, IrqPriority priority) {
    uint32_t prioritygroup = 0x00U;
    prioritygroup = NVIC_GetPriorityGrouping();
    NVIC_EnableIRQRequest(irqn, priority.preempt_pri(), priority.sub_pri()); // FIXME remove after testing
    NVIC_SetPriority(irqn, NVIC_EncodePriority(prioritygroup, priority.preempt_pri(), priority.sub_pri()));
}

inline void enable_irq(IRQn_Type irqn) { NVIC_EnableIRQ(irqn); }
inline void disable_irq(IRQn_Type irqn) { NVIC_DisableIRQ(irqn); }
inline void clear_pending_irq(IRQn_Type irqn) { NVIC_ClearPendingIRQ(irqn); }


inline void enable_interrupts() { __enable_irq(); }


inline void disable_interrupts() { __disable_irq(); }


class critical_section {
private:
    bool irq_enabled;
public:
    critical_section() {
        irq_enabled = (__get_PRIMASK() == 0);
        __disable_irq();
    }

    ~critical_section() {
        if (irq_enabled) {
            __enable_irq();
        }
    }
};


} // namespace mcu


#endif
#endif
