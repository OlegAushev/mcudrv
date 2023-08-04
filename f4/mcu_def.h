#pragma once


#ifdef STM32F4xx

#include "../common.h"
#include <cassert>


namespace mcu {

using HalStatus = HAL_StatusTypeDef;


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

} // namespace mcu

#endif

