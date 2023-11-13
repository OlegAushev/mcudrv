#ifdef MCUDRV_C28X


#include <mcudrv/c28x/f2837xd/spi/spi.h>


namespace mcu {


namespace spi {


const uint32_t impl::spi_bases[3] = {SPIA_BASE, SPIB_BASE, SPIC_BASE};
const uint32_t impl::spi_rx_pie_int_nums[3] = {INT_SPIA_RX, INT_SPIB_RX, INT_SPIC_RX};


Module::Module(Peripheral peripheral,
               const gpio::Config& mosi_pin, const gpio::Config& miso_pin,
               const gpio::Config& clk_pin, const gpio::Config& cs_pin,
               const Config& config)
        : emb::interrupt_invoker_array<Module, peripheral_count>(this, peripheral.underlying_value())
        , _peripheral(peripheral)
        , _module(impl::spi_bases[peripheral.underlying_value()], impl::spi_rx_pie_int_nums[peripheral.underlying_value()]) {
    assert((config.data_size >= 1) && (config.data_size <= 16));

    _word_len = config.word_len;

    SPI_disableModule(_module.base);
    SPI_setConfig(_module.base, DEVICE_LSPCLK_FREQ,
            static_cast<SPI_TransferProtocol>(config.protocol.underlying_value()),
            static_cast<SPI_Mode>(config.mode.underlying_value()),
            static_cast<uint32_t>(config.bitrate.underlying_value()),
            static_cast<uint16_t>(config.word_len.underlying_value()));
    SPI_disableLoopback(_module.base);
    SPI_setEmulationMode(_module.base, SPI_EMULATION_FREE_RUN);
#ifdef CPU1
    _init_pins(mosi_pin, miso_pin, clk_pin, cs_pin);
#endif
    SPI_enableFIFO(_module.base);
    SPI_setFIFOInterruptLevel(_module.base, SPI_FIFO_TXEMPTY, static_cast<SPI_RxFIFOLevel>(config.data_size));
    SPI_enableModule(_module.base);
}


#ifdef CPU1
void Module::transfer_control_to_cpu2(Peripheral peripheral, const gpio::Config& mosi_pin, const gpio::Config& miso_pin,
                                        const gpio::Config& clk_pin, const gpio::Config& cs_pin) {
    _init_pins(mosi_pin, miso_pin, clk_pin, cs_pin);
    GPIO_setMasterCore(mosi_pin.no, GPIO_CORE_CPU2);
    GPIO_setMasterCore(miso_pin.no, GPIO_CORE_CPU2);
    GPIO_setMasterCore(clk_pin.no, GPIO_CORE_CPU2);
    if (cs_pin.valid) {
        GPIO_setMasterCore(cs_pin.no, GPIO_CORE_CPU2);
    }

    SysCtl_selectCPUForPeripheral(SYSCTL_CPUSEL6_SPI, peripheral.underlying_value()+1, SYSCTL_CPUSEL_CPU2);
}
#endif


#ifdef CPU1
void Module::_init_pins(const gpio::Config& mosi_pin, const gpio::Config& miso_pin,
                        const gpio::Config& clk_pin, const gpio::Config& cs_pin) {
    GPIO_setPinConfig(mosi_pin.mux);
    //GPIO_setPadConfig(mosiPin.no, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(mosi_pin.no, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(miso_pin.mux);
    //GPIO_setPadConfig(misoPin.no, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(miso_pin.no, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(clk_pin.mux);
    //GPIO_setPadConfig(clkPin.no, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(clk_pin.no, GPIO_QUAL_ASYNC);

    if (cs_pin.valid) {
        GPIO_setPinConfig(cs_pin.mux);
        //GPIO_setPadConfig(csPin.no, GPIO_PIN_TYPE_PULLUP);
        GPIO_setQualificationMode(cs_pin.no, GPIO_QUAL_ASYNC);
    }
}
#endif


} // namespace spi


} // namespace mcu


#endif
