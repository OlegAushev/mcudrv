#pragma once


#include "../system/mcu_system.h"
#include <dsp/vcu/vcu2_crc.h>


namespace mcu {

namespace crc {


uint32_t calc_crc32(uint16_t* buf, size_t bytes)
{
	CRC_Obj crc_obj;

	crc_obj.seedValue = INIT_CRC32;
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


} // namespace crc

} // namespace mcu

