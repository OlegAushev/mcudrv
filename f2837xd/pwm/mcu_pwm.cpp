#include <mcu_c28x/f2837xd/pwm/mcu_pwm.h>


namespace mcu {

namespace pwm {

const uint32_t impl::pwm_bases[12] = {EPWM1_BASE, EPWM2_BASE, EPWM3_BASE, EPWM4_BASE,
					EPWM5_BASE, EPWM6_BASE, EPWM7_BASE, EPWM8_BASE,
					EPWM9_BASE, EPWM10_BASE, EPWM11_BASE, EPWM12_BASE};
const uint32_t impl::pwm_pie_event_int_nums[12] = {INT_EPWM1, INT_EPWM2, INT_EPWM3, INT_EPWM4,
						INT_EPWM5, INT_EPWM6, INT_EPWM7, INT_EPWM8,
						INT_EPWM9, INT_EPWM10, INT_EPWM11, INT_EPWM12};
const uint32_t impl::pwm_pie_trip_int_nums[12] = {INT_EPWM1_TZ, INT_EPWM2_TZ, INT_EPWM3_TZ, INT_EPWM4_TZ,
						INT_EPWM5_TZ, INT_EPWM6_TZ, INT_EPWM7_TZ, INT_EPWM8_TZ,
						INT_EPWM9_TZ, INT_EPWM10_TZ, INT_EPWM11_TZ, INT_EPWM12_TZ};

} // namespace pwm

} // namespace mcu

