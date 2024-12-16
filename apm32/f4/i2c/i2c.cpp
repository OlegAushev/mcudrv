#ifdef MCUDRV_APM32
#ifdef APM32F4xx


#include <mcudrv/apm32/f4/i2c/i2c.h>


namespace mcu {
namespace i2c {


Module::Module(Peripheral peripheral,
               const SdaPinConfig& sda_pin_config, const SclPinConfig& scl_pin_config,
               const Config& config)
        : emb::singleton_array<Module, peripheral_count>(this, std::to_underlying(peripheral))
        , _peripheral(peripheral)
        , _reg(impl::instances[std::to_underlying(peripheral)])
{
    _sda_pin.init({
        .port = sda_pin_config.port,
        .pin = sda_pin_config.pin,
        .config = {
            .pin{},
            .mode = GPIO_MODE_AF,
            .speed = GPIO_SPEED_50MHz,
            .otype = GPIO_OTYPE_OD,
            .pupd = GPIO_PUPD_NOPULL
        },
        .altfunc = sda_pin_config.altfunc,
        .active_state{}});

    _scl_pin.init({
        .port = scl_pin_config.port,
        .pin = scl_pin_config.pin,
        .config = {
            .pin{},
            .mode = GPIO_MODE_AF,
            .speed = GPIO_SPEED_50MHz,
            .otype = GPIO_OTYPE_OD,
            .pupd = GPIO_PUPD_NOPULL
        },
        .altfunc = scl_pin_config.altfunc,
        .active_state{}});

    _enable_clk(peripheral);

    _cfg = config;
    auto i2c_config = config.hal_config;
    I2C_Config(_reg, &i2c_config);
}


void Module::_enable_clk(Peripheral peripheral) {
    size_t i2c_idx = std::to_underlying(peripheral);
    if (_clk_enabled[i2c_idx]) {
        return;
    }

    impl::clk_enable_funcs[i2c_idx]();
    _clk_enabled[i2c_idx] = true;
}


} // namespace i2c
} // namespace mcu


#endif
#endif
