#pragma once


#if !(defined MCUDRV_C28X) && !(defined MCUDRV_STM32) && !(defined MCUDRV_APM32)
#error "mcudrv error: mcu is not defined!"
#endif
