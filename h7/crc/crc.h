#pragma once


#ifdef STM32H7xx


#include "../mcu_def.h"
#include "../system/system.h"


namespace mcu {


namespace crc {


enum InputDataFormat {
    bytes = CRC_INPUTDATA_FORMAT_BYTES,
    halfwords = CRC_INPUTDATA_FORMAT_HALFWORDS,
    words = CRC_INPUTDATA_FORMAT_WORDS
};


struct Config {
    CRC_InitTypeDef hal_init;
    InputDataFormat input_data_format;
};


class calc_unit {
private:
    calc_unit() = default;
    static inline CRC_HandleTypeDef _handle;
public:
    calc_unit(const calc_unit& other) = delete;
    calc_unit& operator=(const calc_unit& other) = delete;

    static void init(const Config& config) {
        _handle.Instance = CRC;
        _handle.Init = config.hal_init;
        _handle.InputDataFormat = static_cast<uint32_t>(config.input_data_format);
        if (HAL_CRC_Init(&_handle) != HAL_OK)
        {
            fatal_error("CRC module initialization failed");
        }
    }

    static HalStatus deinit() { return HAL_CRC_DeInit(&_handle); }
    static uint32_t calculate(uint32_t* buf, uint32_t len) { return HAL_CRC_Calculate(&_handle, buf, len); }
    static uint32_t accumulate(uint32_t* buf, uint32_t len) { return HAL_CRC_Accumulate(&_handle, buf, len); }
};


} // namespace crc


} // namespace mcu


#endif
