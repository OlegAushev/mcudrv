#pragma once


#ifdef MCUDRV_STM32
#ifdef STM32H7xx


#include <mcudrv/stm32/h7/stm32_h7_base.h>
#include <mcudrv/stm32/h7/system/system.h>
#include "../gpio/gpio.h"
#include <utility>


namespace mcu {


namespace i2c {


constexpr size_t peripheral_count = 4;
enum class Peripheral : unsigned int {
    i2c1,
    i2c2,
    i2c3,
    i2c4
};


struct SdaPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct SclPinConfig {
    GPIO_TypeDef* port;
    uint32_t pin;
    uint32_t af_selection;
};


struct Config {
    I2C_InitTypeDef hal_config;
};


namespace impl {


inline const std::array<I2C_TypeDef*, peripheral_count> i2c_instances = {I2C1, I2C2, I2C3, I2C4};


inline Peripheral to_peripheral(const I2C_TypeDef* instance) {
    return static_cast<Peripheral>(
        std::distance(i2c_instances.begin(), std::find(i2c_instances.begin(), i2c_instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> uart_clk_enable_funcs = {
    [](){ __HAL_RCC_I2C1_CLK_ENABLE(); },
    [](){ __HAL_RCC_I2C2_CLK_ENABLE(); },
    [](){ __HAL_RCC_I2C3_CLK_ENABLE(); },
    [](){ __HAL_RCC_I2C4_CLK_ENABLE(); },
};


} // namespace impl


class Module : public emb::singleton_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    I2C_HandleTypeDef _handle = {};
    I2C_TypeDef* _reg;

    mcu::gpio::AlternatePin _sda_pin;
    mcu::gpio::AlternatePin _scl_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled = {};
public:
    Module(Peripheral peripheral, const SdaPinConfig& sda_pin_config, const SclPinConfig& scl_pin_config, const Config& config);
    
    Peripheral peripheral() const { return _peripheral; }
    I2C_HandleTypeDef* handle() { return &_handle; }
    I2C_TypeDef* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::singleton_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void set_slave_address(uint16_t slave_addr) { modify_reg<uint32_t>(_reg->CR2, I2C_CR2_SADD, slave_addr); }
    void enable() { set_bit<uint32_t>(_reg->CR1, I2C_CR1_PE); }
    void disable() { clear_bit<uint32_t>(_reg->CR1, I2C_CR1_PE); }

    bool busy() const { return bit_is_set<uint32_t>(_reg->ISR, I2C_ISR_BUSY); }
    bool stop_detected() const { return bit_is_set<uint32_t>(_reg->ISR, I2C_ISR_STOPF); }
    
    DrvStatus write_mem(uint16_t devaddr, uint16_t memaddr, uint16_t memaddrsize, const uint8_t *data, size_t len, std::chrono::milliseconds timeout) {
        return static_cast<DrvStatus>(
                HAL_I2C_Mem_Write(&_handle, devaddr, memaddr, memaddrsize, const_cast<uint8_t*>(data), uint16_t(len), uint32_t(timeout.count())));
    }

    DrvStatus read_mem(uint16_t devaddr, uint16_t memaddr, uint16_t memaddrsize, uint8_t *data, size_t len, std::chrono::milliseconds timeout) {
        return static_cast<DrvStatus>(
                HAL_I2C_Mem_Read(&_handle, devaddr, memaddr, memaddrsize, data, uint16_t(len), uint32_t(timeout.count())));
    }


    
private:
    static void _enable_clk(Peripheral peripheral);
};















} // namespace i2c


} // namespace mcu


#endif
#endif
