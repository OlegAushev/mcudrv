#pragma once


#include <cstdint>
#include <cmsis_gcc.h>


namespace mcu {

inline void set_bit(uint32_t& reg, uint32_t bit) { reg |= bit; }

inline void clear_bit(uint32_t& reg, uint32_t bit) { reg &= ~bit; }

inline uint32_t read_bit(uint32_t reg, uint32_t bit) { return reg & bit; }

inline void clear_reg(uint32_t& reg) { reg = 0; }

template <typename T>
void write_reg(volatile T& reg, T val) { reg = val; }

inline uint32_t read_reg(uint32_t reg) { return reg; }

inline void modify_reg(uint32_t& reg, uint32_t clearmask, uint32_t setmask) { reg = (reg & ~clearmask) | setmask; }

inline uint32_t position_val(uint32_t val) { return __CLZ(__RBIT(val)); }

}
