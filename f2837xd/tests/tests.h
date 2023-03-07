#pragma once


#include <emblib_c28x/testrunner/testrunner.h>
#include "../system/system.h"
#include "mculib_c28x/f2837xd/gpio/gpio.h"
#include "mculib_c28x/f2837xd/chrono/chrono.h"
#include "mculib_c28x/f2837xd/crc/crc.h"

#ifdef _LAUNCHXL_F28379D
#include <bsp/launchpad/launchpad.h>
#endif


namespace mcu {

class tests
{
public:
	static void gpio();
	static void chrono();
	static void crc();
};

} // namespace mcu

