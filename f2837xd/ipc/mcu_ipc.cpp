#include <mcu_c28x/f2837xd/ipc/mcu_ipc.h>


namespace mcu {

namespace ipc {

namespace flags {

#ifdef DUALCORE
const mcu::ipc::Mode mode = mcu::ipc::Mode::singlecore;
#else
const mcu::ipc::Mode mode = mcu::ipc::Mode::dualcore;
#endif

mcu::ipc::Flag cpu1_periphery_configured(31, mode);
mcu::ipc::Flag cpu2_booted(30, mode);
mcu::ipc::Flag cpu2_periphery_configured(29, mode);

} // namespace flags

} // namespace ipc

} // namespace mcu

