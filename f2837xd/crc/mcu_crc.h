#pragma once


#include "../system/mcu_system.h"
#include <dsp/vcu/vcu2_crc.h>


namespace mcu {

namespace crc {

inline uint32_t calc_crc32(uint16_t* buf, size_t bytes)
{
	// CRC-32/MPEG-2
	CRC_Obj crc_obj;

	crc_obj.seedValue = 0xFFFFFFFF;//INIT_CRC32;
	crc_obj.nMsgBytes = bytes;
	crc_obj.parity = CRC_parity_even;
	crc_obj.crcResult = 0;
	crc_obj.pMsgBuffer = buf;
	crc_obj.init = reinterpret_cast<void(*)(void*)>(CRC_init32Bit);
	crc_obj.run = reinterpret_cast<void(*)(void*)>(CRC_run32BitPoly1);

	crc_obj.init(&crc_obj);
	crc_obj.run(&crc_obj);

	return crc_obj.crcResult;
}


inline void reset()
{
	//CRC_reset();
	uint16_t buf[4] = {0x0100, 0x0302, 0x0504, 0x0706};
	uint32_t res = calc_crc32(buf, 8);
	EMB_UNUSED(res);
}

} // namespace crc

} // namespace mcu

