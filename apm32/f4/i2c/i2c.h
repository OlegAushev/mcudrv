#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/apm32_base.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <apm32f4xx_i2c.h>


namespace mcu {
namespace i2c {


constexpr size_t peripheral_count = 3;
enum class Peripheral : unsigned int {
    i2c1,
    i2c2,
    i2c3
};


enum class Direction { rx, tx };


struct SdaPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };
struct SclPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T altfunc; };


struct Config {
    I2C_Config_T hal_config;
};


namespace impl {


inline const std::array<I2C_T*, peripheral_count> instances = {I2C1, I2C2, I2C3};


inline Peripheral to_peripheral(const I2C_T* instance) {
    return static_cast<Peripheral>(
        std::distance(instances.begin(), std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C1); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C2); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_I2C3); },
};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    I2C_T* const _reg;
    mcu::gpio::AlternatePin _sda_pin;
    mcu::gpio::AlternatePin _scl_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral,
           const SdaPinConfig& sda_pin_config, const SclPinConfig& scl_pin_config,
           const Config& config);

    Peripheral peripheral() const { return _peripheral; }
    I2C_T* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

    void enable() { _reg->CTRL1_B.I2CEN = 1; }
    void disable() { _reg->CTRL1_B.I2CEN = 0; }

    void toggle_start(bool v = true) { _reg->CTRL1_B.START = v; }
    void toggle_stop(bool v = true) { _reg->CTRL1_B.STOP = v; }

    void put_addr(uint8_t addr, Direction dir) {
        if (dir == Direction::rx) {
            _reg->DATA_B.DATA = addr | 0x01;
        } else {
            _reg->DATA_B.DATA = addr & 0xFE;
        }
    }

    

    // bool busy() const { return _reg->STS_B.BSYFLG == 1; }
    // bool rx_empty() const { return _reg->STS_B.RXBNEFLG == 0; }
    // bool tx_empty() const { return _reg->STS_B.TXBEFLG == 1; }

    // DrvStatus put_data(uint16_t data) {
    //     if (_reg->STS_B.TXBEFLG == 0) {
    //         return DrvStatus::busy;
    //     }
    //     _reg->DATA_B.DATA = data;
    //     return DrvStatus::ok;
    // }

    // std::optional<uint16_t> get_data() {
    //     if (_reg->STS_B.RXBNEFLG == 0) {
    //         return {};
    //     }
    //     uint16_t data = _reg->DATA_B.DATA;
    //     return {data};
    // }

    // void set_bidirectional_mode(Direction dir) {
    //     switch (dir) {
    //         case Direction::rx:
    //             _reg->CTRL1_B.BMOEN = 0;
    //             break;
    //         case Direction::tx:
    //             _reg->CTRL1_B.BMOEN = 1;
    //             break;
    //     }
    // }

    // void set_cs(size_t cs_idx = 0) {
    //     if (cs_idx >= _cs_pins.size()) {
    //         return;
    //     }
    //     _cs_pins[cs_idx].set();
    // }

    // void reset_cs(size_t cs_idx = 0) {
    //     if (cs_idx >= _cs_pins.size()) {
    //         return;
    //     }
    //     _cs_pins[cs_idx].reset();
    // }
protected:
    static void _enable_clk(Peripheral peripheral);
};














} // namespace i2c
} // namespace mcu


#endif
#endif
