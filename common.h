#pragma once


#include <cstdint>

#if defined(STM32F4xx)
#include <stm32f4xx.h>
#elif defined(STM32H7xx)
#include "stm32h7xx.h"
#endif


namespace mcu {


template <typename T>
bool is_bit_set(const volatile T& reg, T bit) { return (reg & bit) == bit; }


template <typename T>
bool is_bit_clr(const volatile T& reg, T bit) { return (reg & bit) == 0; }


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


inline uint32_t position_val(uint32_t val) { return __CLZ(__RBIT(val)); }


class InterruptPriority {
private:
    uint32_t _value;
public:
    explicit InterruptPriority(uint32_t value)
            : _value(value) {
        assert_param(value <= 15);
    }

    uint32_t get() const { return _value; }
};


inline void set_interrupt_priority(IRQn_Type irqn, InterruptPriority priority) { HAL_NVIC_SetPriority(irqn, priority.get(), 0);}
inline void enable_interrupt(IRQn_Type irqn) { HAL_NVIC_EnableIRQ(irqn); }
inline void disable_interrupt(IRQn_Type irqn) { HAL_NVIC_DisableIRQ(irqn); }


}
