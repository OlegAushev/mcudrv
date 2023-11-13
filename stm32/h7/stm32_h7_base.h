#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../common.h"
#include <cassert>


namespace mcu {


using HalStatus = HAL_StatusTypeDef;


} // namespace mcu


#endif
#endif
