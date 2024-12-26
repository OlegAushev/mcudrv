#ifdef MCUDRV_APM32
#ifdef APM32F4xx

#include <mcudrv/apm32/f4/spi/spi.h>

namespace mcu {
namespace apm32 {
namespace spi {

Module::Module(Peripheral peripheral,
               const MosiPinConfig& mosi_pin_config,
               const MisoPinConfig& miso_pin_config,
               const ClkPinConfig& clk_pin_config,
               const HwCsPinConfig& cs_pin_config,
               const Config& config)
        : emb::singleton_array<Module, peripheral_count>(
                  this, std::to_underlying(peripheral)),
          _peripheral(peripheral),
          _reg(impl::instances[std::to_underlying(peripheral)]) {
    _init_mosi_miso_clk(mosi_pin_config, miso_pin_config, clk_pin_config);

    _cs_pin.init({.port = cs_pin_config.port,
                  .pin = cs_pin_config.pin,
                  .config = {.pin{},
                             .mode = GPIO_MODE_AF,
                             .speed = GPIO_SPEED_25MHz,
                             .otype = GPIO_OTYPE_PP,
                             .pupd = GPIO_PUPD_NOPULL},
                  .altfunc = cs_pin_config.altfunc,
                  .active_state = mcu::gpio::active_state::low});

    _enable_clk(peripheral);

    auto spi_config = config.hal_config;
    SPI_Config(_reg, &spi_config);
    SPI_Enable(_reg);
}

Module::Module(Peripheral peripheral,
               const MosiPinConfig& mosi_pin_config,
               const MisoPinConfig& miso_pin_config,
               const ClkPinConfig& clk_pin_config,
               std::initializer_list<SwCsPinConfig> cs_pin_configs,
               const Config& config)
        : emb::singleton_array<Module, peripheral_count>(
                  this, std::to_underlying(peripheral)),
          _peripheral(peripheral),
          _reg(impl::instances[std::to_underlying(peripheral)]) {
    if (cs_pin_configs.size() != 0 &&
        config.hal_config.mode != SPI_MODE_MASTER) {
        fatal_error();
    }

    _init_mosi_miso_clk(mosi_pin_config, miso_pin_config, clk_pin_config);

    for (auto pincfg : cs_pin_configs) {
        _cs_pins.emplace_back(gpio::OutputPin(
                gpio::PinConfig{.port = pincfg.port,
                                .pin = pincfg.pin,
                                .config = {.pin{},
                                           .mode = GPIO_MODE_OUT,
                                           .speed = GPIO_SPEED_25MHz,
                                           .otype = GPIO_OTYPE_PP,
                                           .pupd = GPIO_PUPD_UP},
                                .altfunc{},
                                .active_state = mcu::gpio::active_state::low}));
        _cs_pins.back().reset();
    }

    _enable_clk(peripheral);

    auto spi_config = config.hal_config;
    SPI_Config(_reg, &spi_config);
}

void Module::init_interrupts(std::initializer_list<InterruptEvent> events,
                             IrqPriority priority) {
    for (auto event : events) {
        switch (event) {
        case InterruptEvent::txe:
            _reg->CTRL2_B.TXBEIEN = 1;
            break;
        case InterruptEvent::rxne:
            _reg->CTRL2_B.RXBNEIEN = 1;
            break;
        case InterruptEvent::err:
            _reg->CTRL2_B.ERRIEN = 1;
            break;
        }
    }
    set_irq_priority(impl::irqn[std::to_underlying(_peripheral)], priority);
}

void Module::_enable_clk(Peripheral peripheral) {
    size_t spi_idx = std::to_underlying(peripheral);
    if (_clk_enabled[spi_idx]) {
        return;
    }

    impl::clk_enable_funcs[spi_idx]();
    _clk_enabled[spi_idx] = true;
}

void Module::_init_mosi_miso_clk(const MosiPinConfig& mosi_pin_config,
                                 const MisoPinConfig& miso_pin_config,
                                 const ClkPinConfig& clk_pin_config) {
    _mosi_pin.init({.port = mosi_pin_config.port,
                    .pin = mosi_pin_config.pin,
                    .config = {.pin{},
                               .mode = GPIO_MODE_AF,
                               .speed = GPIO_SPEED_100MHz,
                               .otype = GPIO_OTYPE_PP,
                               .pupd = GPIO_PUPD_NOPULL},
                    .altfunc = mosi_pin_config.altfunc,
                    .active_state = mcu::gpio::active_state::high});

    _miso_pin.init({.port = miso_pin_config.port,
                    .pin = miso_pin_config.pin,
                    .config = {.pin{},
                               .mode = GPIO_MODE_AF,
                               .speed = GPIO_SPEED_100MHz,
                               .otype = GPIO_OTYPE_PP,
                               .pupd = GPIO_PUPD_NOPULL},
                    .altfunc = miso_pin_config.altfunc,
                    .active_state = mcu::gpio::active_state::high});

    _clk_pin.init({.port = clk_pin_config.port,
                   .pin = clk_pin_config.pin,
                   .config = {.pin{},
                              .mode = GPIO_MODE_AF,
                              .speed = GPIO_SPEED_100MHz,
                              .otype = GPIO_OTYPE_PP,
                              .pupd = GPIO_PUPD_NOPULL},
                   .altfunc = clk_pin_config.altfunc,
                   .active_state = mcu::gpio::active_state::high});
}

} // namespace spi
} // namespace apm32
} // namespace mcu

#endif
#endif
