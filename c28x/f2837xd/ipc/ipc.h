#pragma once

#ifdef MCUDRV_C28X

#include "../system/system.h"
#include <emblib/core.h>
#include "F2837xD_Ipc_drivers.h"

namespace mcu {
namespace c28x {
namespace ipc {

SCOPED_ENUM_DECLARE_BEGIN(Cpu) {
    cpu1,
    cpu2,
} SCOPED_ENUM_DECLARE_END(Cpu)

#if defined(CPU1)
const Cpu this_cpu = Cpu::cpu1;
#elif defined(CPU2)
const Cpu this_cpu = Cpu::cpu2;
#endif

namespace newimpl {

class Flag {
private:
    uint32_t bitmask_;
    Cpu master_cpu_;
public:
    Flag(uint32_t flag_id, Cpu master_cpu)
            : bitmask_(1 << flag_id), master_cpu_(master_cpu) {
        assert(flag_id < 32);
    }

    void set() {
        assert(this_cpu == master_cpu_);
    }
};


} // namespace newimpl



namespace traits {
struct singlecore{};
struct dualcore{};
struct primary{};
struct secondary{};
}


SCOPED_ENUM_DECLARE_BEGIN(Mode) {
    singlecore,
    dualcore
} SCOPED_ENUM_DECLARE_END(Mode)


SCOPED_ENUM_DECLARE_BEGIN(Role) {
    primary,
    secondary
} SCOPED_ENUM_DECLARE_END(Role)


class LocalFlag {
private:
    uint32_t _mask;
public:
    LocalFlag() : _mask(0) {}
    explicit LocalFlag(uint32_t flagNo)
            : _mask(1UL << flagNo) {
        assert(flagNo < 32);
    }

    void init(uint32_t flagNo) {
        assert(flagNo < 32);
        _mask = 1UL << flagNo;
    }

    void set() { IPCLtoRFlagSet(_mask); }
    void reset() { IPCLtoRFlagClear(_mask); }
    bool is_set() const { return IPCLtoRFlagBusy(_mask); }
};


class RemoteFlag {
private:
    uint32_t _mask;
public:
    RemoteFlag() : _mask(0) {}
    explicit RemoteFlag(uint32_t flagNo)
            : _mask(1UL << flagNo) {
        assert(flagNo < 32);
    }

    void init(uint32_t flagNo) {
        assert(flagNo < 32);
        _mask = 1UL << flagNo;
    }

    void wait() {
        while(!IPCRtoLFlagBusy(_mask));
        IPCRtoLFlagAcknowledge(_mask);
    }

    bool is_set() const { return IPCRtoLFlagBusy(_mask); }
    void acknowledge() { IPCRtoLFlagAcknowledge(_mask); }
};


class Flag {
private:
    Mode _mode;
public:
    LocalFlag local;
    RemoteFlag remote;
    Flag() {}
    explicit Flag(uint32_t flagNo, Mode mode)
            : _mode(mode)
            , local(flagNo)
            , remote(flagNo) {}

    void init(uint32_t flagNo, Mode mode) {
        _mode = mode;
        local.init(flagNo);
        remote.init(flagNo);
    }

    bool is_set() const {
        switch (_mode.native_value()) {
        case mcu::c28x::ipc::Mode::singlecore:
            return local.is_set();
        case mcu::c28x::ipc::Mode::dualcore:
            return remote.is_set();
        }
        return false;
    }

    void reset() {
        switch (_mode.native_value()) {
        case mcu::c28x::ipc::Mode::singlecore:
            local.reset();
            return;
        case mcu::c28x::ipc::Mode::dualcore:
            remote.acknowledge();
            return;
        }
    }

    Mode mode() const { return _mode; }
};


SCOPED_ENUM_DECLARE_BEGIN(InterruptType) {
    ipc_interrupt0 = INT_IPC_0,
    ipc_interrupt1 = INT_IPC_1,
    ipc_interrupt2 = INT_IPC_2,
    ipc_interrupt3 = INT_IPC_3,
} SCOPED_ENUM_DECLARE_END(InterruptType)


inline void registerIpcInterruptHandler(InterruptType ipc_interrupt, void (*handler)(void)) {
    Interrupt_register(ipc_interrupt.underlying_value(), handler);
    Interrupt_enable(ipc_interrupt.underlying_value());
}


namespace flags {


//extern mcu::c28x::ipc::Flag cpu1_periphery_configured;
//extern mcu::c28x::ipc::Flag cpu2_booted;
//extern mcu::c28x::ipc::Flag cpu2_periphery_configured;


} // namespace flags


} // namespace ipc

} // namespace c28x

} // namespace mcu


#endif
