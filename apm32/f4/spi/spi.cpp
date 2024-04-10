#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/spi/spi.h>


namespace mcu {
namespace spi {


Module::Module(Peripheral peripheral,
               const MosiPinConfig& mosi_pin_config, const MisoPinConfig& miso_pin_config,
               const ClkPinConfig& clk_pin_config, std::optional<HwCsPinConfig> cs_pin_config,
               const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _reg(impl::instances[std::to_underlying(peripheral)])
{
    _init_mosi_miso_clk(mosi_pin_config, miso_pin_config, clk_pin_config);

    if (cs_pin_config.has_value()) {
        _cs_pin.initialize({.port = cs_pin_config.value().port,
                                .pin = {.pin = cs_pin_config.value().pin,
                                        .mode = GPIO_MODE_AF,
                                        .speed = GPIO_SPEED_25MHz,
                                        .otype = GPIO_OTYPE_PP,
                                        .pupd = GPIO_PUPD_NOPULL},
                                .altfunc = cs_pin_config.value().altfunc,
                                .actstate = emb::gpio::active_pin_state::low});
    }

    _enable_clk(peripheral);

    auto spi_config = config.hal_config;
    SPI_Config(_reg, &spi_config);
    SPI_Enable(_reg);
}


Module::Module(Peripheral peripheral,
               const MosiPinConfig& mosi_pin_config, const MisoPinConfig& miso_pin_config,
               const ClkPinConfig& clk_pin_config, const std::vector<SwCsPinConfig>& cs_pin_configs,
               const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _reg(impl::instances[std::to_underlying(peripheral)])
{
    _init_mosi_miso_clk(mosi_pin_config, miso_pin_config, clk_pin_config);

    for (const auto& pincfg : cs_pin_configs) {
        _cs_pins.emplace_back(mcu::gpio::OutputPin(mcu::gpio::Config{.port = pincfg.port,
                .pin = {.pin = pincfg.pin,
                        .mode = GPIO_MODE_OUT,
                        .speed = GPIO_SPEED_25MHz,
                        .otype = GPIO_OTYPE_PP,
                        .pupd = GPIO_PUPD_UP},
                .altfunc{},
                .actstate = emb::gpio::active_pin_state::low}));
    }
    
    _enable_clk(peripheral);

    auto spi_config = config.hal_config;
    SPI_Config(_reg, &spi_config);
    SPI_Enable(_reg);
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
    _mosi_pin.initialize({.port = mosi_pin_config.port,
                            .pin = {.pin = mosi_pin_config.pin,
                                    .mode = GPIO_MODE_AF,
                                    .speed = GPIO_SPEED_100MHz,
                                    .otype = GPIO_OTYPE_PP,
                                    .pupd = GPIO_PUPD_NOPULL},
                            .altfunc = mosi_pin_config.altfunc,
                            .actstate = emb::gpio::active_pin_state::high});

    _miso_pin.initialize({.port = miso_pin_config.port,
                            .pin = {.pin = miso_pin_config.pin,
                                    .mode = GPIO_MODE_AF,
                                    .speed = GPIO_SPEED_100MHz,
                                    .otype = GPIO_OTYPE_PP,
                                    .pupd = GPIO_PUPD_NOPULL},
                            .altfunc = miso_pin_config.altfunc,
                            .actstate = emb::gpio::active_pin_state::high});

    _clk_pin.initialize({.port = clk_pin_config.port,
                            .pin = {.pin = clk_pin_config.pin,
                                    .mode = GPIO_MODE_AF,
                                    .speed = GPIO_SPEED_100MHz,
                                    .otype = GPIO_OTYPE_PP,
                                    .pupd = GPIO_PUPD_NOPULL},
                            .altfunc = clk_pin_config.altfunc,
                            .actstate = emb::gpio::active_pin_state::high});
}


} // namespace spi
} // namespace mcu


#endif
#endif
