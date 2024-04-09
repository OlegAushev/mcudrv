#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/spi/spi.h>


namespace mcu {
namespace spi {


Module::Module(Peripheral peripheral,
               const MosiPinConfig& mosi_pin_config, const MisoPinConfig& miso_pin_config,
               const ClkPinConfig& clk_pin_config, std::optional<CsPinConfig> cs_pin_config,
               const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _reg(impl::instances[std::to_underlying(peripheral)])
{
    _mosi_pin.initialize({.port = mosi_pin_config.port,
                            .pin = {.pin = mosi_pin_config.pin,
                                    .mode = GPIO_MODE_AF,
                                    .speed = GPIO_SPEED_100MHz,
                                    .otype = GPIO_OTYPE_PP,
                                    .pupd = GPIO_PUPD_NOPULL},
                            .af_selection = mosi_pin_config.af_selection,
                            .actstate = emb::gpio::active_pin_state::high});

    _miso_pin.initialize({.port = miso_pin_config.port,
                            .pin = {.pin = miso_pin_config.pin,
                                    .mode = GPIO_MODE_AF,
                                    .speed = GPIO_SPEED_100MHz,
                                    .otype = GPIO_OTYPE_PP,
                                    .pupd = GPIO_PUPD_NOPULL},
                            .af_selection = miso_pin_config.af_selection,
                            .actstate = emb::gpio::active_pin_state::high});

    _clk_pin.initialize({.port = clk_pin_config.port,
                            .pin = {.pin = clk_pin_config.pin,
                                    .mode = GPIO_MODE_AF,
                                    .speed = GPIO_SPEED_100MHz,
                                    .otype = GPIO_OTYPE_PP,
                                    .pupd = GPIO_PUPD_NOPULL},
                            .af_selection = clk_pin_config.af_selection,
                            .actstate = emb::gpio::active_pin_state::high});

    if (cs_pin_config.has_value()) {
        _cs_pin.initialize({.port = cs_pin_config.value().port,
                                .pin = {.pin = cs_pin_config.value().pin,
                                        .mode = GPIO_MODE_AF,
                                        .speed = GPIO_SPEED_100MHz,
                                        .otype = GPIO_OTYPE_PP,
                                        .pupd = GPIO_PUPD_NOPULL},
                                .af_selection = cs_pin_config.value().af_selection,
                                .actstate = emb::gpio::active_pin_state::high});
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


} // namespace spi
} // namespace mcu


#endif
#endif