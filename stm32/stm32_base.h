#pragma once


#ifdef MCUDRV_STM32


#include <mcudrv/config.h>
#include <cstdint>
#include <cassert>


#if defined(STM32F4xx)
#include <stm32f4xx.h>
#elif defined(STM32H7xx)
#include <stm32h7xx.h>
#endif


namespace mcu {


enum class bit_type {
    rw,
    r,
    w,
    rc_w0,
    rc_w1,
    rc_w,
    rc_r,
    rs_r,
    rs,
    rwo,
    t,
    rt_w1
};


namespace impl {


template <bit_type B>
concept BitClearableW0 = ( B == bit_type::rw || B == bit_type::rc_w0 || B == bit_type::rc_w );


template <bit_type B>
concept BitClearableW1 = ( B == bit_type::rc_w1 );


}


template <typename T>
bool bit_is_set(const volatile T& reg, T bit) { return (reg & bit) == bit; }


template <typename T>
bool bit_is_clear(const volatile T& reg, T bit) { return (reg & bit) == 0; }


template <typename T>
void set_bit(volatile T& reg, T bit) { reg |= bit; }


template <typename T, bit_type B = bit_type::rw>
void clear_bit(volatile T& reg, T bit) requires impl::BitClearableW0<B> { reg &= ~bit; }


template <typename T, bit_type B = bit_type::rw>
void clear_bit(volatile T& reg, T bit) requires impl::BitClearableW1<B> { reg |= bit; }


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


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
inline uint32_t position_val(uint32_t val) { return __builtin_clz(__RBIT(val)); }
#pragma GCC diagnostic pop


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


inline void set_irq_priority(IRQn_Type irqn, IrqPriority priority) { HAL_NVIC_SetPriority(irqn, priority.get(), 0);}
inline void enable_irq(IRQn_Type irqn) { HAL_NVIC_EnableIRQ(irqn); }
inline void disable_irq(IRQn_Type irqn) { HAL_NVIC_DisableIRQ(irqn); }
inline void clear_pending_irq(IRQn_Type irqn) { HAL_NVIC_ClearPendingIRQ(irqn); }


} // namespace mcu


#endif
