#pragma once


#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/apm32_base.h>
#include <mcudrv/apm32/f4/system/system.h>
#include <mcudrv/apm32/f4/gpio/gpio.h>
#include <apm32f4xx_spi.h>
#include <optional>


namespace mcu {
namespace spi {


constexpr size_t peripheral_count = 3;
enum class Peripheral : unsigned int {
    spi1,
    spi2,
    spi3
};


struct MosiPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T af_selection; };
struct MisoPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T af_selection; };
struct ClkPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T af_selection; };
struct CsPinConfig { GPIO_T* port; uint16_t pin; GPIO_AF_T af_selection; };


struct Config {
    SPI_Config_T hal_config;
};


namespace impl {


inline const std::array<SPI_T*, peripheral_count> instances = {SPI1, SPI2, SPI3};


inline Peripheral to_peripheral(const SPI_T* instance) {
    return static_cast<Peripheral>(
        std::distance(instances.begin(), std::find(instances.begin(), instances.end(), instance)));
}


inline std::array<void(*)(void), peripheral_count> clk_enable_funcs = {
    [](){ RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SPI1); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI2); },
    [](){ RCM_EnableAPB1PeriphClock(RCM_APB1_PERIPH_SPI3); },
};


} // namespace impl


class Module : public emb::interrupt_invoker_array<Module, peripheral_count>, private emb::noncopyable {
private:
    const Peripheral _peripheral;
    SPI_T* const _reg;
    mcu::gpio::AlternatePin _mosi_pin;
    mcu::gpio::AlternatePin _miso_pin;
    mcu::gpio::AlternatePin _clk_pin;
    mcu::gpio::AlternatePin _cs_pin;

    static inline std::array<bool, peripheral_count> _clk_enabled{};
public:
    Module(Peripheral peripheral,
           const MosiPinConfig& mosi_pin_config, const MisoPinConfig& miso_pin_config,
           const ClkPinConfig& clk_pin_config, std::optional<CsPinConfig> cs_pin_config,
           const Config& config);
    Peripheral peripheral() const { return _peripheral; }
    SPI_T* reg() { return _reg; }

    static Module* instance(Peripheral peripheral) {
        return emb::interrupt_invoker_array<Module, peripheral_count>::instance(std::to_underlying(peripheral));
    }

protected:
    static void _enable_clk(Peripheral peripheral);
};





} // namespace spi
} // namespace mcu


#endif
#endif