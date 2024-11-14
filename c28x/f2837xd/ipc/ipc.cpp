#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/ipc/ipc.h>


namespace mcu {

namespace c28x {

namespace ipc {


namespace flags {


#ifdef DUALCORE
const mcu::c28x::ipc::Mode mode = mcu::c28x::ipc::Mode::singlecore;
#else
const mcu::c28x::ipc::Mode mode = mcu::c28x::ipc::Mode::dualcore;
#endif

mcu::c28x::ipc::Flag cpu1_periphery_configured(31, mode);
mcu::c28x::ipc::Flag cpu2_booted(30, mode);
mcu::c28x::ipc::Flag cpu2_periphery_configured(29, mode);


} // namespace flags


} // namespace ipc

} // namespace c28x

} // namespace mcu


#endif
