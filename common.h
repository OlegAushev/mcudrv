#pragma once


#include <cstdint>
#include <cmsis_gcc.h>


namespace mcu {

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

}
