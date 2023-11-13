#pragma once


#include <cassert>
#include <cstdint>


#ifdef APM32F4xx
#include <apm32f4xx.h>
#endif


namespace mcu {


template <typename T>
bool bit_is_set(const volatile T& reg, T bit) { return (reg & bit) == bit; }


template <typename T>
bool bit_is_clear(const volatile T& reg, T bit) { return (reg & bit) == 0; }


template <typename T>
void set_bit(volatile T& reg, T bit) { reg |= bit; }


template <typename T>
void clear_bit(volatile T& reg, T bit) { reg &= ~bit; }


template <typename T>
T read_bit(const volatile T& reg, T bit) { return reg & bit; }


template <typename T>
void clear_reg(volatile T& reg) { reg = 0; }


template <typename T>
void write_reg(volatile T& reg, T val) { reg = val; }


template <typename T>
T read_reg(const volatile T& reg) { return reg; }


template <typename T>
void modify_reg(volatile T& reg, T clearmask, T setmask) { reg = (reg & ~clearmask) | setmask; }


inline uint32_t bit_position(uint32_t val) { return __CLZ(__RBIT(val)); }


class IrqPriority {
private:
    uint32_t _value;
public:
    explicit IrqPriority(uint32_t value)
            : _value(value) {
        assert(value <= 15);
    }

    uint32_t get() const { return _value; }
};

// TODO
// inline void set_irq_priority(IRQn_Type irqn, IrqPriority priority) { HAL_NVIC_SetPriority(irqn, priority.get(), 0);}
// inline void enable_irq(IRQn_Type irqn) { HAL_NVIC_EnableIRQ(irqn); }
// inline void disable_irq(IRQn_Type irqn) { HAL_NVIC_DisableIRQ(irqn); }
// inline void clear_pending_irq(IRQn_Type irqn) { HAL_NVIC_ClearPendingIRQ(irqn); }


}
