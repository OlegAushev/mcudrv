#pragma once


#include "../system/system.h"
#include <c28x_emblib/core.h>
#include "F2837xD_Ipc_drivers.h"


namespace mcu {

namespace ipc {

namespace traits {
struct singlecore{};
struct dualcore{};
struct primary{};
struct secondary{};
}


SCOPED_ENUM_DECLARE_BEGIN(Mode)
{
	singlecore,
	dualcore
}
SCOPED_ENUM_DECLARE_END(Mode)


SCOPED_ENUM_DECLARE_BEGIN(Role)
{
	primary,
	secondary
}
SCOPED_ENUM_DECLARE_END(Role)


class LocalFlag
{
private:
	uint32_t _mask;
public:
	LocalFlag() : _mask(0) {}
	explicit LocalFlag(uint32_t flagNo)
		: _mask(1UL << flagNo)
	{
		assert(flagNo < 32);
	}

	void init(uint32_t flagNo)
	{
		assert(flagNo < 32);
		_mask = 1UL << flagNo;
	}

	void set() { IPCLtoRFlagSet(_mask); }
	inline void reset() { IPCLtoRFlagClear(_mask); }
	bool is_set() { return IPCLtoRFlagBusy(_mask); }
};


class RemoteFlag
{
private:
	uint32_t _mask;
public:
	RemoteFlag() : _mask(0) {}
	explicit RemoteFlag(uint32_t flagNo)
		: _mask(1UL << flagNo)
	{
		assert(flagNo < 32);
	}

	void init(uint32_t flagNo)
	{
		assert(flagNo < 32);
		_mask = 1UL << flagNo;
	}

	void wait()
	{
		while(!IPCRtoLFlagBusy(_mask));
		IPCRtoLFlagAcknowledge(_mask);
	}

	bool is_set() { return IPCRtoLFlagBusy(_mask); }
	inline void acknowledge() { IPCRtoLFlagAcknowledge(_mask); }
};


class Flag
{
private:
	Mode _mode;
public:
	LocalFlag local;
	RemoteFlag remote;
	Flag() {}
	explicit Flag(uint32_t flagNo, Mode mode)
		: _mode(mode)
		, local(flagNo)
		, remote(flagNo)
	{}

	void init(uint32_t flagNo, Mode mode)
	{
		_mode = mode;
		local.init(flagNo);
		remote.init(flagNo);
	}

	bool is_set()
	{
		switch (_mode.native_value())
		{
			case mcu::ipc::Mode::singlecore:
				return local.is_set();
			case mcu::ipc::Mode::dualcore:
				return remote.is_set();
		}
		return false;
	}

	inline void reset()
	{
		switch (_mode.native_value())
		{
			case mcu::ipc::Mode::singlecore:
				local.reset();
				return;
			case mcu::ipc::Mode::dualcore:
				remote.acknowledge();
				return;
		}
	}
};


SCOPED_ENUM_DECLARE_BEGIN(InterruptType)
{
	ipc_interrupt0 = INT_IPC_0,
	ipc_interrupt1 = INT_IPC_1,
	ipc_interrupt2 = INT_IPC_2,
	ipc_interrupt3 = INT_IPC_3,
}
SCOPED_ENUM_DECLARE_END(InterruptType)


inline void registerIpcInterruptHandler(InterruptType ipc_interrupt, void (*handler)(void))
{
	Interrupt_register(ipc_interrupt.underlying_value(), handler);
	Interrupt_enable(ipc_interrupt.underlying_value());
}


namespace flags {

extern mcu::ipc::Flag cpu1_periphery_configured;
extern mcu::ipc::Flag cpu2_booted;
extern mcu::ipc::Flag cpu2_periphery_configured;

} // namespace flags

} // namespace ipc

} // namespace mcu

