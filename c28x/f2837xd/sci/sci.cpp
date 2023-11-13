#ifdef MCUDRV_C28X

#include <mcudrv/c28x/f2837xd/sci/sci.h>


namespace mcu {


namespace sci {


const uint32_t impl::sci_bases[4] = {SCIA_BASE, SCIB_BASE, SCIC_BASE, SCID_BASE};
const uint32_t impl::sci_rx_pie_int_nums[4] = {INT_SCIA_RX, INT_SCIB_RX, INT_SCIC_RX, INT_SCID_RX};
const uint16_t impl::sci_pie_int_groups[4] = {
    INTERRUPT_ACK_GROUP9, INTERRUPT_ACK_GROUP9, INTERRUPT_ACK_GROUP8, INTERRUPT_ACK_GROUP8
};


Module::Module(Peripheral peripheral, const gpio::Config& rx_pin, const gpio::Config& tx_pin, const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
        , _peripheral(peripheral)
        , _module(impl::sci_bases[peripheral.underlying_value()],
                  impl::sci_rx_pie_int_nums[peripheral.underlying_value()],
                  impl::sci_pie_int_groups[peripheral.underlying_value()]) {
#ifdef CPU1
    _init_pins(rx_pin, tx_pin);
#endif
    SCI_disableModule(_module.base);

    uint32_t config_flags = static_cast<uint32_t>(config.word_len.underlying_value())
                          | static_cast<uint32_t>(config.stop_bits.underlying_value())
                          | static_cast<uint32_t>(config.parity_mode.underlying_value());

    SCI_setConfig(_module.base, DEVICE_LSPCLK_FREQ,
                  static_cast<uint32_t>(config.baudrate.underlying_value()),
                  config_flags);

    SCI_resetChannels(_module.base);
    SCI_resetRxFIFO(_module.base);
    SCI_resetTxFIFO(_module.base);

    SCI_clearInterruptStatus(_module.base, SCI_INT_TXFF | SCI_INT_RXFF);
    SCI_setFIFOInterruptLevel(_module.base, SCI_FIFO_TX8, SCI_FIFO_RX8);
    SCI_enableFIFO(_module.base);
    SCI_enableModule(_module.base);
    SCI_performSoftwareReset(_module.base);

    if (config.autobaud_mode == AutoBaudMode::enabled) {
        // Perform an autobaud lock.
        // SCI expects an 'a' or 'A' to lock the baud rate.
        SCI_lockAutobaud(_module.base);
    }
}


#ifdef CPU1
void Module::transfer_control_to_cpu2(Peripheral peripheral, const gpio::Config& rx_pin, const gpio::Config& tx_pin) {
    _init_pins(rx_pin, tx_pin);
    GPIO_setMasterCore(rx_pin.no, GPIO_CORE_CPU2);
    GPIO_setMasterCore(tx_pin.no, GPIO_CORE_CPU2);
    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL5_SCI,
            peripheral.underlying_value()+1, SYSCTL_CPUSEL_CPU2);
}
#endif


#ifdef CPU1
void Module::_init_pins(const gpio::Config& rx_pin, const gpio::Config& tx_pin) {
    GPIO_setPinConfig(rx_pin.mux);
    GPIO_setDirectionMode(rx_pin.no, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(rx_pin.no, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(rx_pin.no, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(tx_pin.mux);
    GPIO_setDirectionMode(tx_pin.no, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(tx_pin.no, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(tx_pin.no, GPIO_QUAL_ASYNC);
}
#endif


} // namespace sci


} // namespace mcu


#endif
