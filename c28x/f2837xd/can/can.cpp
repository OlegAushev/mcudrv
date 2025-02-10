#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/can/can.hpp>

namespace mcu {
namespace c28x {

namespace can {

const uint32_t impl::can_bases[2] = {CANA_BASE, CANB_BASE};
const uint32_t impl::can_pie_int_nums[2] = {INT_CANA0, INT_CANB0};
const SysCtl_CPUSelPeriphInstance impl::can_cpusel_instances[2] = {
        SYSCTL_CPUSEL_CANA,
        SYSCTL_CPUSEL_CANB};

void (*Module::on_interrupt_callbacks_[peripheral_count])(Module*,
                                                          uint32_t,
                                                          uint16_t);

Module::Module(Peripheral peripheral,
               const RxPinConfig& rx_pin,
               const TxPinConfig& tx_pin,
               const Config& config)
        : emb::singleton_array<Module, peripheral_count>(
                this, peripheral.underlying_value()),
          peripheral_(peripheral),
          module_(impl::can_bases[peripheral.underlying_value()],
                  impl::can_pie_int_nums[peripheral.underlying_value()]) {
#ifdef CPU1
    init_pins(rx_pin, tx_pin);
#endif

    CAN_initModule(module_.base);
    CAN_selectClockSource(module_.base, CAN_CLOCK_SOURCE_SYS);

    switch (config.bitrate.native_value()) {
    case Bitrate::bitrate_125k:
    case Bitrate::bitrate_250k:
        CAN_setBitRate(module_.base,
                       mcu::c28x::sysclk_freq(),
                       static_cast<uint32_t>(config.bitrate.underlying_value()),
                       16);
        break;
    case Bitrate::bitrate_500k:
    case Bitrate::bitrate_1000k:
        CAN_setBitRate(module_.base,
                       mcu::c28x::sysclk_freq(),
                       static_cast<uint32_t>(config.bitrate.underlying_value()),
                       10);
        break;
    }

    CAN_setAutoBusOnTime(module_.base, 0);
    CAN_enableAutoBusOn(module_.base);

    if (config.mode != Mode::normal) {
        CAN_enableTestMode(module_.base, config.mode.underlying_value());
    }

    CAN_startModule(module_.base);
}

#ifdef CPU1
void Module::transfer_control_to_cpu2(Peripheral peripheral,
                                      const RxPinConfig& rx_pin,
                                      const TxPinConfig& tx_pin) {
    init_pins(rx_pin, tx_pin);
    GPIO_setMasterCore(rx_pin.pin, GPIO_CORE_CPU2);
    GPIO_setMasterCore(tx_pin.pin, GPIO_CORE_CPU2);
    SysCtl_selectCPUForPeripheralInstance(
            impl::can_cpusel_instances[peripheral.underlying_value()],
            SYSCTL_CPUSEL_CPU2);
}
#endif

void Module::register_interrupt_handler(void (*handler)(void)) {
    Interrupt_register(module_.pie_int_num, handler);
    CAN_enableInterrupt(module_.base,
                        CAN_INT_IE0 | CAN_INT_ERROR | CAN_INT_STATUS);
    CAN_enableGlobalInterrupt(module_.base, CAN_GLOBAL_INT_CANINT0);
}

void Module::register_interrupt_callback(void (*callback)(Module*,
                                                          uint32_t,
                                                          uint16_t)) {
    switch (peripheral_.native_value()) {
    case Peripheral::cana:
        register_interrupt_handler(on_interrupt<Peripheral::cana>);
        break;
    case Peripheral::canb:
        register_interrupt_handler(on_interrupt<Peripheral::canb>);
        break;
    }
    on_interrupt_callbacks_[peripheral_.underlying_value()] = callback;
}

#ifdef CPU1
void Module::init_pins(const RxPinConfig& rx_pin, const TxPinConfig& tx_pin) {
    GPIO_setPinConfig(rx_pin.mux);
    GPIO_setPinConfig(tx_pin.mux);
}
#endif

} // namespace can
} // namespace c28x
} // namespace mcu

#endif
