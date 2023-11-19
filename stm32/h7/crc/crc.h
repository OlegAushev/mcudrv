#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include "../stm32_h7_base.h"
#include "../system/system.h"


namespace mcu {


namespace crc {


inline void init() {
    __HAL_RCC_CRC_CLK_ENABLE();
}


inline uint8_t calc_crc8(const uint8_t* buf, size_t len) {
    CRC_HandleTypeDef handle = {.Instance = CRC,
                                .Init = {.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE,
                                         .DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE,
                                         .GeneratingPolynomial = 0x07,
                                         .CRCLength = CRC_POLYLENGTH_8B,
                                         .InitValue = 0x00,
                                         .InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE,
                                         .OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE},
                                .Lock{},
                                .State{},
                                .InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES};
    
    if (HAL_CRC_Init(&handle) != HAL_OK) {
        return 0;
    }

    return HAL_CRC_Calculate(&handle, reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(buf)), len);
}


inline uint32_t calc_crc32(const uint8_t* buf, size_t len) {
    // MPEG-2
    CRC_HandleTypeDef handle = {.Instance = CRC,
                                .Init = {.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE,
                                         .DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE,
                                         .GeneratingPolynomial = 0x04C11DB7,
                                         .CRCLength = CRC_POLYLENGTH_32B,
                                         .InitValue = 0xFFFFFFFF,
                                         .InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE,
                                         .OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE},
                                .Lock{},
                                .State{},
                                .InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES};
    
    if (HAL_CRC_Init(&handle) != HAL_OK) {
        return 0;
    }

    return HAL_CRC_Calculate(&handle, reinterpret_cast<uint32_t*>(const_cast<uint8_t*>(buf)), len);
}


} // namespace crc


} // namespace mcu


#endif
#endif
