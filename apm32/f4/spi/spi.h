#pragma once

#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <apm32f4xx_spi.h>

#include <mcudrv/apm32/apm32_base.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <mcudrv/apm32/f4/system/system.h>

#include <initializer_list>
#include <optional>
#include <vector>

namespace mcu {
namespace apm32 {
namespace spi {

constexpr size_t peripheral_count = 3;
enum class Peripheral : unsigned int { spi1, spi2, spi3 };

enum class Direction { rx, tx };

struct MosiPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};
struct MisoPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};
struct ClkPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};
struct HwCsPinConfig {
    gpio::Port port;
    gpio::Pin pin;
    GPIO_AF_T altfunc;
};
struct SwCsPinConfig {
    gpio::Port port;
    gpio::Pin pin;
};

struct Config {
    SPI_Config_T hal_config;
};

enum class InterruptEvent { txe, rxne, err };

namespace impl {

inline const std::array<SPI_T*, peripheral_count> instances = {
    SPI1, SPI2, SPI3};

inline Peripheral to_peripheral(const SPI_T* instance) {
    return static_cast<Peripheral>(std::distance(
            instances.begin(),
            std::find(instances.begin(), instances.end(), instance)));
}

inline std::array<void (*)(void), peripheral_count> clk_enable_funcs = {
    []() { RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SPI1); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2); },
    []() { RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI3); },
};

inline constexpr std::array<IRQn_Type, peripheral_count> irqn = {
    SPI1_IRQn, SPI2_IRQn, SPI3_IRQn};

} // namespace impl

class Module : public emb::singleton_array<Module, peripheral_count>,
               private emb::noncopyable {
private:
    const Peripheral _peripheral;
    SPI_T* const _reg;
    mcu::gpio::AlternatePin _mosi_pin;
    mcu::gpio::AlternatePin _miso_pin;
    mcu::gpio::AlternatePin _clk_pin;
    mcu::gpio::AlternatePin _cs_pin;
    std::vector<mcu::gpio::OutputPin> _cs_pins;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral,
           const MosiPinConfig& mosi_pin_config,
           const MisoPinConfig& miso_pin_config,
           const ClkPinConfig& clk_pin_config,
           const HwCsPinConfig& cs_pin_config,
           const Config& config);

    Module(Peripheral peripheral,
           const MosiPinConfig& mosi_pin_config,
           const MisoPinConfig& miso_pin_config,
           const ClkPinConfig& clk_pin_config,
           std::initializer_list<SwCsPinConfig> cs_pin_configs,
           const Config& config);

    Peripheral peripheral() const { return _peripheral; }
    SPI_T* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::singleton_array<Module, peripheral_count>::instance(
                std::to_underlying(peripheral));
    }

    void enable() { _reg->CTRL1_B.SPIEN = 1; }
    void disable() { _reg->CTRL1_B.SPIEN = 0; }

    bool busy() const { return _reg->STS_B.BSYFLG == 1; }
    bool rx_empty() const { return _reg->STS_B.RXBNEFLG == 0; }
    bool tx_empty() const { return _reg->STS_B.TXBEFLG == 1; }

    exec_status put_data(uint16_t data) {
        if (!tx_empty()) {
            return exec_status::busy;
        }
        _reg->DATA_B.DATA = data;
        return exec_status::ok;
    }

    std::optional<uint16_t> get_data() const {
        if (rx_empty()) {
            return {};
        }
        uint16_t data = _reg->DATA_B.DATA;
        return {data};
    }

    void set_bidirectional_mode(Direction dir) {
        switch (dir) {
        case Direction::rx:
            _reg->CTRL1_B.BMOEN = 0;
            break;
        case Direction::tx:
            _reg->CTRL1_B.BMOEN = 1;
            break;
        }
    }

    void set_cs(size_t cs_idx = 0) {
        if (cs_idx >= _cs_pins.size()) {
            return;
        }
        _cs_pins[cs_idx].set();
    }

    void reset_cs(size_t cs_idx = 0) {
        if (cs_idx >= _cs_pins.size()) {
            return;
        }
        _cs_pins[cs_idx].reset();
    }
public:
    void init_interrupts(std::initializer_list<InterruptEvent> events,
                         IrqPriority priority);
    void enable_interrupts() {
        enable_irq(impl::irqn[std::to_underlying(_peripheral)]);
    }
    void disable_interrupts() {
        disable_irq(impl::irqn[std::to_underlying(_peripheral)]);
    }
protected:
    static void _enable_clk(Peripheral peripheral);
    void _init_mosi_miso_clk(const MosiPinConfig& mosi_pin_config,
                             const MisoPinConfig& miso_pin_config,
                             const ClkPinConfig& clk_pin_config);
};

} // namespace spi
} // namespace apm32
} // namespace mcu

#endif
#endif
